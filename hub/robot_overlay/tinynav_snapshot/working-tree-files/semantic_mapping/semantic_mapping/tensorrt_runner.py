"""Small synchronous TensorRT runner isolated from semantic postprocessing."""

from __future__ import annotations

from dataclasses import dataclass
import ctypes
import importlib
from pathlib import Path
import platform
from typing import Any, Mapping

import numpy as np
from numpy.typing import NDArray


class _CudaRuntime:
    """Minimal libcudart bindings needed for synchronous TensorRT execution."""

    HOST_ALLOC_MAPPED = 0x02
    MEMCPY_HOST_TO_DEVICE = 1
    MEMCPY_DEVICE_TO_HOST = 2

    def __init__(self) -> None:
        try:
            self.library = ctypes.CDLL("libcudart.so")
        except OSError as error:
            raise RuntimeError("TensorRT backend cannot load libcudart.so") from error
        self.library.cudaGetErrorString.argtypes = [ctypes.c_int]
        self.library.cudaGetErrorString.restype = ctypes.c_char_p
        self.library.cudaStreamCreate.argtypes = [ctypes.POINTER(ctypes.c_void_p)]
        self.library.cudaStreamCreate.restype = ctypes.c_int
        self.library.cudaStreamDestroy.argtypes = [ctypes.c_void_p]
        self.library.cudaStreamDestroy.restype = ctypes.c_int
        self.library.cudaStreamSynchronize.argtypes = [ctypes.c_void_p]
        self.library.cudaStreamSynchronize.restype = ctypes.c_int
        self.library.cudaHostAlloc.argtypes = [
            ctypes.POINTER(ctypes.c_void_p),
            ctypes.c_size_t,
            ctypes.c_uint,
        ]
        self.library.cudaHostAlloc.restype = ctypes.c_int
        self.library.cudaHostGetDevicePointer.argtypes = [
            ctypes.POINTER(ctypes.c_void_p),
            ctypes.c_void_p,
            ctypes.c_uint,
        ]
        self.library.cudaHostGetDevicePointer.restype = ctypes.c_int
        self.library.cudaMallocHost.argtypes = [
            ctypes.POINTER(ctypes.c_void_p),
            ctypes.c_size_t,
        ]
        self.library.cudaMallocHost.restype = ctypes.c_int
        self.library.cudaMalloc.argtypes = [
            ctypes.POINTER(ctypes.c_void_p),
            ctypes.c_size_t,
        ]
        self.library.cudaMalloc.restype = ctypes.c_int
        self.library.cudaFreeHost.argtypes = [ctypes.c_void_p]
        self.library.cudaFreeHost.restype = ctypes.c_int
        self.library.cudaFree.argtypes = [ctypes.c_void_p]
        self.library.cudaFree.restype = ctypes.c_int
        self.library.cudaMemcpyAsync.argtypes = [
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_size_t,
            ctypes.c_int,
            ctypes.c_void_p,
        ]
        self.library.cudaMemcpyAsync.restype = ctypes.c_int

    def create_stream(self) -> int:
        stream = ctypes.c_void_p()
        self._check(self.library.cudaStreamCreate(ctypes.byref(stream)), "stream create")
        return int(stream.value)

    def destroy_stream(self, stream: int) -> None:
        self._check(
            self.library.cudaStreamDestroy(ctypes.c_void_p(stream)),
            "stream destroy",
        )

    def synchronize(self, stream: int) -> None:
        self._check(
            self.library.cudaStreamSynchronize(ctypes.c_void_p(stream)),
            "stream synchronize",
        )

    def allocate_mapped_host(self, nbytes: int) -> tuple[int, int]:
        host = ctypes.c_void_p()
        self._check(
            self.library.cudaHostAlloc(
                ctypes.byref(host), nbytes, self.HOST_ALLOC_MAPPED
            ),
            "mapped host allocation",
        )
        device = ctypes.c_void_p()
        self._check(
            self.library.cudaHostGetDevicePointer(
                ctypes.byref(device), host, 0
            ),
            "mapped device pointer",
        )
        return int(host.value), int(device.value)

    def allocate_host_and_device(self, nbytes: int) -> tuple[int, int]:
        host = ctypes.c_void_p()
        device = ctypes.c_void_p()
        self._check(
            self.library.cudaMallocHost(ctypes.byref(host), nbytes),
            "host allocation",
        )
        self._check(
            self.library.cudaMalloc(ctypes.byref(device), nbytes),
            "device allocation",
        )
        return int(host.value), int(device.value)

    def free_host(self, pointer: int) -> None:
        self._check(
            self.library.cudaFreeHost(ctypes.c_void_p(pointer)), "free host"
        )

    def free_device(self, pointer: int) -> None:
        self._check(
            self.library.cudaFree(ctypes.c_void_p(pointer)), "free device"
        )

    def copy_async(
        self, destination: int, source: int, nbytes: int, kind: int, stream: int
    ) -> None:
        self._check(
            self.library.cudaMemcpyAsync(
                ctypes.c_void_p(destination),
                ctypes.c_void_p(source),
                nbytes,
                kind,
                ctypes.c_void_p(stream),
            ),
            "asynchronous memory copy",
        )

    def _check(self, result: int, operation: str) -> None:
        if result == 0:
            return
        raw_message = self.library.cudaGetErrorString(result)
        message = raw_message.decode("utf-8") if raw_message else "unknown error"
        raise RuntimeError(f"CUDA {operation} failed ({result}): {message}")


@dataclass
class _TensorBuffer:
    name: str
    shape: tuple[int, ...]
    dtype: np.dtype
    host: NDArray
    host_pointer: int
    device_pointer: int
    mapped: bool

    @property
    def nbytes(self) -> int:
        return int(self.host.nbytes)


class TensorRtEngineRunner:
    """Execute one static-profile TensorRT engine with synchronous NumPy I/O."""

    def __init__(
        self,
        engine_path: str | Path,
        input_shapes: Mapping[str, tuple[int, ...]],
    ) -> None:
        try:
            self.trt = importlib.import_module("tensorrt")
        except ImportError as error:
            raise RuntimeError(
                "TensorRT backend requires the local tensorrt Python module"
            ) from error
        self.cuda = _CudaRuntime()

        source = Path(engine_path).expanduser()
        if not source.is_file():
            raise ValueError(f"TensorRT engine does not exist: {source}")
        self.logger = self.trt.Logger(self.trt.Logger.WARNING)
        with source.open("rb") as stream, self.trt.Runtime(self.logger) as runtime:
            self.engine = runtime.deserialize_cuda_engine(stream.read())
        if self.engine is None:
            raise RuntimeError(f"Failed to deserialize TensorRT engine: {source}")
        self.context = self.engine.create_execution_context()
        if self.context is None:
            raise RuntimeError("Failed to create TensorRT execution context")
        self._closed = False
        self._mapped = platform.machine() == "aarch64"
        self.stream = self.cuda.create_stream()

        engine_input_names = {
            self.engine.get_tensor_name(index)
            for index in range(self.engine.num_io_tensors)
            if self.engine.get_tensor_mode(self.engine.get_tensor_name(index))
            == self.trt.TensorIOMode.INPUT
        }
        if set(input_shapes) != engine_input_names:
            raise ValueError(
                f"Engine inputs {sorted(engine_input_names)} do not match "
                f"configured inputs {sorted(input_shapes)}"
            )
        for name, shape in input_shapes.items():
            if not self.context.set_input_shape(name, tuple(int(v) for v in shape)):
                raise ValueError(f"TensorRT rejected input shape for {name}: {shape}")

        self.inputs: dict[str, _TensorBuffer] = {}
        self.outputs: dict[str, _TensorBuffer] = {}
        self._buffers: list[_TensorBuffer] = []
        for index in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(index)
            shape = tuple(int(value) for value in self.context.get_tensor_shape(name))
            if any(value <= 0 for value in shape):
                raise ValueError(f"TensorRT tensor {name} has unresolved shape {shape}")
            dtype = np.dtype(self.trt.nptype(self.engine.get_tensor_dtype(name)))
            buffer = self._allocate_buffer(name, shape, dtype)
            if not self.context.set_tensor_address(name, int(buffer.device_pointer)):
                raise RuntimeError(f"Failed to bind TensorRT tensor: {name}")
            self._buffers.append(buffer)
            if self.engine.get_tensor_mode(name) == self.trt.TensorIOMode.INPUT:
                self.inputs[name] = buffer
            else:
                self.outputs[name] = buffer

    @property
    def input_shapes(self) -> dict[str, tuple[int, ...]]:
        return {name: buffer.shape for name, buffer in self.inputs.items()}

    @property
    def output_shapes(self) -> dict[str, tuple[int, ...]]:
        return {name: buffer.shape for name, buffer in self.outputs.items()}

    def infer(self, inputs: Mapping[str, NDArray]) -> dict[str, NDArray]:
        """Copy named arrays, execute the engine, and return owned output arrays."""
        if self._closed:
            raise RuntimeError("TensorRT runner is closed")
        if set(inputs) != set(self.inputs):
            raise ValueError("TensorRT input names do not match engine inputs")
        for name, raw_value in inputs.items():
            buffer = self.inputs[name]
            value = np.asarray(raw_value)
            if value.shape != buffer.shape:
                raise ValueError(
                    f"TensorRT input {name} shape {value.shape} != {buffer.shape}"
                )
            np.copyto(buffer.host, value, casting="safe")
            if not buffer.mapped:
                self.cuda.copy_async(
                    buffer.device_pointer,
                    buffer.host.ctypes.data,
                    buffer.nbytes,
                    self.cuda.MEMCPY_HOST_TO_DEVICE,
                    self.stream,
                )

        if not self.context.execute_async_v3(stream_handle=self.stream):
            raise RuntimeError("TensorRT execute_async_v3 failed")
        if not self._mapped:
            for buffer in self.outputs.values():
                self.cuda.copy_async(
                    buffer.host.ctypes.data,
                    buffer.device_pointer,
                    buffer.nbytes,
                    self.cuda.MEMCPY_DEVICE_TO_HOST,
                    self.stream,
                )
        self.cuda.synchronize(self.stream)
        return {name: buffer.host.copy() for name, buffer in self.outputs.items()}

    def close(self) -> None:
        """Release CUDA host/device allocations and the execution stream."""
        if self._closed:
            return
        for buffer in self._buffers:
            if not buffer.mapped:
                self.cuda.free_device(buffer.device_pointer)
            self.cuda.free_host(buffer.host_pointer)
        self.cuda.destroy_stream(self.stream)
        self._closed = True

    def _allocate_buffer(
        self, name: str, shape: tuple[int, ...], dtype: np.dtype
    ) -> _TensorBuffer:
        ctypes_types: dict[np.dtype, Any] = {
            np.dtype(np.float32): ctypes.c_float,
            np.dtype(np.float16): ctypes.c_uint16,
            np.dtype(np.int8): ctypes.c_int8,
            np.dtype(np.uint8): ctypes.c_uint8,
            np.dtype(np.int32): ctypes.c_int32,
            np.dtype(np.int64): ctypes.c_int64,
            np.dtype(np.bool_): ctypes.c_bool,
        }
        if dtype not in ctypes_types:
            raise ValueError(f"Unsupported TensorRT NumPy dtype: {dtype}")
        size = int(np.prod(shape, dtype=np.int64))
        nbytes = size * dtype.itemsize
        if self._mapped:
            host_pointer, device_pointer = self.cuda.allocate_mapped_host(nbytes)
        else:
            host_pointer, device_pointer = self.cuda.allocate_host_and_device(nbytes)
        ctype = ctypes_types[dtype]
        host = np.ctypeslib.as_array((ctype * size).from_address(host_pointer))
        host = host.view(dtype).reshape(shape)
        return _TensorBuffer(
            name=name,
            shape=shape,
            dtype=dtype,
            host=host,
            host_pointer=int(host_pointer),
            device_pointer=int(device_pointer),
            mapped=self._mapped,
        )
