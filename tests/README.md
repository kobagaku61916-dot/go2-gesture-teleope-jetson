# tests

移植時に旧リポから以下をそのまま持ってくる（import パスのみ変更。ロジック等価性の保証が目的）:

- `test_gesture_mapper.py` — 12 件（旧 gesture_ope.computeCommand との等価性を含む）
- `test_dance_detector.py` — 7 件（交互腕伸ばしパターンの発火・cooldown・リセット）

新規追加予定:
- 信頼度チェック（visibility 下限 → NO BODY 化）の単体テスト
- debounce（N フレーム連続一致で確定）の単体テスト

実行: `venv/bin/python3 -m pytest tests/ -q`（mediapipe 不要のテスト構成を維持する）
