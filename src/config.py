"""configs/params.yaml のローダ.

本リポは ament パッケージにしない（venv python 直接実行）ため、
ROS パラメータではなく YAML を直接読む。セクション単位で辞書を返す。
"""

from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_CONFIG = REPO_ROOT / "configs" / "params.yaml"


def load_section(section: str, path=None) -> dict:
    """params.yaml の指定セクションを辞書で返す.

    Args:
        section: "gesture_node" / "safety_gate" / "go2_bridge"。
        path: 設定ファイルパス（省略時は configs/params.yaml）。
    """
    p = Path(path) if path else DEFAULT_CONFIG
    with open(p, encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if section not in data:
        raise KeyError(f"{p} にセクション {section!r} がありません")
    return data[section]
