"""
Step 4 — Frontier Output Format + Parser + Decision Logger

1. Patch the prompt so the VLM outputs the decision letter FIRST.
2. Parse and validate the VLM's output letter (A/B/C/D) against actual frontier candidates.
3. Log every VLM decision call with full context for DoD verification.
4. Write a separate human-readable log alongside the JSONL.
"""

import re
import json
import logging
import os
from datetime import datetime
from dataclasses import dataclass, field, asdict
from typing import Optional
from pathlib import Path


# ═══════════════════════════════════════════════════════════
# 1. PROMPT PATCH — force decision-first output
# ═══════════════════════════════════════════════════════════

def _build_decision_suffix(valid_candidates: list) -> str:
    """Build the output-format instruction using only the actual candidate letters."""
    if valid_candidates:
        letters = ", ".join(valid_candidates[:-1]) + (
            f", or {valid_candidates[-1]}" if len(valid_candidates) > 1
            else valid_candidates[0]
        )
    else:
        letters = "A, B, C, or D"

    return f"""
**CRITICAL OUTPUT FORMAT — follow EXACTLY:**
Line 1: Your choice letter ONLY ({letters}).
Line 2: REASON=<one short sentence>

Example:
{valid_candidates[0] if valid_candidates else 'B'}
REASON=closest frontier to kitchen, unobstructed path

Output now:
"""


def patch_frontier_prompt(original_prompt: str, valid_candidates: list = None) -> str:
    """
    Replace MCoCoNav's original output instruction block with a
    decision-first format so the first generated token is the decision letter.
    """
    if valid_candidates is None:
        valid_candidates = ["A", "B", "C", "D"]
    cut_marker = "Explanation Ends."
    idx = original_prompt.find(cut_marker)
    if idx != -1:
        base = original_prompt[:idx + len(cut_marker)]
    else:
        base = re.sub(
            r'\*\*Output Format:\*\*.*', '', original_prompt, flags=re.DOTALL
        )
    return base.rstrip() + "\n\n" + _build_decision_suffix(valid_candidates)


# ═══════════════════════════════════════════════════════════
# 2. PARSER — robust extraction + validation
# ═══════════════════════════════════════════════════════════

@dataclass
class FrontierParseResult:
    success: bool
    chosen: Optional[str] = None
    reason: Optional[str] = None
    raw_output: str = ""
    error: Optional[str] = None
    fell_back: bool = False


_DECISION_FIRST_RE = re.compile(r'^\s*([A-D])\b', re.MULTILINE)
_DECISION_ANYWHERE_RE = re.compile(r'\b([A-D])\b')
_REASON_RE = re.compile(r'REASON\s*=\s*(.+)', re.IGNORECASE)


def parse_frontier_decision(
    raw_output: str,
    valid_candidates: list,
    fallback: str = "first_valid",
) -> FrontierParseResult:
    """
    Parse the VLM's raw text output and extract a validated frontier letter.

    Parameters
    ----------
    raw_output : str
        Raw VLM generation.
    valid_candidates : list[str]
        The frontier letters actually presented, e.g. ['A', 'B', 'C'].
    fallback : str
        "first_valid" — if parse fails, pick the first valid candidate.
        "none" — just fail.
    """
    raw = raw_output.strip()
    valid_set = set(c.upper() for c in valid_candidates)

    if not valid_set:
        return FrontierParseResult(
            success=False, raw_output=raw, error="valid_candidates is empty"
        )

    reason_match = _REASON_RE.search(raw)
    reason = reason_match.group(1).strip() if reason_match else None

    # Primary: decision-first format
    m = _DECISION_FIRST_RE.search(raw)
    if m and m.group(1) in valid_set:
        return FrontierParseResult(
            success=True, chosen=m.group(1), reason=reason, raw_output=raw
        )

    # Secondary: standalone letter anywhere
    for m in _DECISION_ANYWHERE_RE.finditer(raw):
        if m.group(1) in valid_set:
            return FrontierParseResult(
                success=True, chosen=m.group(1), reason=reason, raw_output=raw,
                error=f"letter not on first line, found '{m.group(1)}' at pos {m.start()}"
            )

    # Fallback
    if fallback == "first_valid" and valid_candidates:
        pick = valid_candidates[0]
        return FrontierParseResult(
            success=True, chosen=pick, reason=reason, raw_output=raw,
            error=f"no valid letter found, fell back to '{pick}'",
            fell_back=True
        )

    return FrontierParseResult(
        success=False, reason=reason, raw_output=raw,
        error=f"no valid letter found in output, valid={list(valid_set)}"
    )


# ═══════════════════════════════════════════════════════════
# 3. DECISION LOGGER — JSONL + human-readable log
# ═══════════════════════════════════════════════════════════

@dataclass
class DecisionLogEntry:
    timestamp: str
    step: int
    agent_id: int
    stage: str  # "perception", "judgment", "frontier", "history"
    valid_candidates: list
    raw_vlm_output: str
    parsed_choice: Optional[str]
    parse_success: bool
    fell_back: bool
    error: Optional[str]
    scores: Optional[dict] = None


class DecisionLogger:
    """
    Logs every VLM decision to:
      1. A JSONL file (machine-readable, for DoD verification)
      2. A plain-text log file (human-readable, for quick inspection)
    """

    def __init__(self, log_dir: str, agent_id: int = 0):
        self.agent_id = agent_id
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.jsonl_path = self.log_dir / f"agent{agent_id}_{ts}.jsonl"
        self.txt_path = self.log_dir / f"agent{agent_id}_{ts}.log"

        self.entries: list = []
        self._jsonl_file = open(self.jsonl_path, "a")
        self._txt_file = open(self.txt_path, "a")

        header = f"[DecisionLogger] agent={agent_id} started at {ts}"
        logging.info(header)
        self._txt_file.write(header + "\n" + "=" * 70 + "\n")
        self._txt_file.flush()

    def log(
        self,
        step: int,
        stage: str,
        valid_candidates: list,
        raw_vlm_output: str,
        parse_result: FrontierParseResult,
        scores: Optional[dict] = None,
    ):
        entry = DecisionLogEntry(
            timestamp=datetime.now().isoformat(),
            step=step,
            agent_id=self.agent_id,
            stage=stage,
            valid_candidates=valid_candidates,
            raw_vlm_output=raw_vlm_output,
            parsed_choice=parse_result.chosen,
            parse_success=parse_result.success,
            fell_back=parse_result.fell_back,
            error=parse_result.error,
            scores=scores,
        )
        self.entries.append(entry)

        # JSONL output
        self._jsonl_file.write(
            json.dumps(asdict(entry), ensure_ascii=False) + "\n"
        )
        self._jsonl_file.flush()

        # Human-readable output
        status = "OK" if parse_result.success and not parse_result.fell_back else (
            "FALLBACK" if parse_result.fell_back else "FAIL"
        )
        lines = [
            f"\n[Step {step}] agent={self.agent_id} stage={stage} [{status}]",
            f"  candidates : {valid_candidates}",
            f"  raw_output : {raw_vlm_output[:200]}{'...' if len(raw_vlm_output) > 200 else ''}",
            f"  chosen     : {parse_result.chosen}",
            f"  reason     : {parse_result.reason}",
        ]
        if parse_result.error:
            lines.append(f"  error      : {parse_result.error}")
        if scores:
            lines.append(f"  scores     : {scores}")
        txt_block = "\n".join(lines) + "\n"
        self._txt_file.write(txt_block)
        self._txt_file.flush()

    def get_dod_report(self) -> dict:
        """
        Compute DoD metrics over all logged entries.

        Pass criteria:
        - format_success_rate >= 99.9%
        - out_of_range_rate == 0%
        - fallback_rate < 1%
        """
        if not self.entries:
            return {"status": "NO_DATA", "total": 0}

        total = len(self.entries)
        decision_entries = [
            e for e in self.entries if e.stage in ("frontier", "history")
        ]
        d_total = len(decision_entries)
        if d_total == 0:
            return {"status": "NO_DECISION_ENTRIES", "total": total}

        successes = sum(1 for e in decision_entries if e.parse_success)
        fallbacks = sum(1 for e in decision_entries if e.fell_back)
        out_of_range = sum(
            1 for e in decision_entries
            if e.parsed_choice and e.parsed_choice not in e.valid_candidates
        )

        format_success_rate = successes / d_total
        fallback_rate = fallbacks / d_total
        out_of_range_rate = out_of_range / d_total

        passed = (
            format_success_rate >= 0.999
            and out_of_range_rate == 0
            and fallback_rate < 0.01
        )

        report = {
            "status": "PASS" if passed else "FAIL",
            "total_calls": total,
            "decision_calls": d_total,
            "format_success_rate": round(format_success_rate, 4),
            "fallback_rate": round(fallback_rate, 4),
            "out_of_range_rate": round(out_of_range_rate, 4),
            "dod_thresholds": {
                "format_success_rate": ">= 0.999",
                "out_of_range_rate": "== 0",
                "fallback_rate": "< 0.01",
            },
        }

        # Also write report to the text log
        self._txt_file.write("\n" + "=" * 70 + "\n")
        self._txt_file.write("DoD REPORT\n")
        self._txt_file.write(json.dumps(report, indent=2) + "\n")
        self._txt_file.write("=" * 70 + "\n")
        self._txt_file.flush()

        return report

    def close(self):
        if self._jsonl_file:
            self._jsonl_file.close()
        if self._txt_file:
            self._txt_file.close()
