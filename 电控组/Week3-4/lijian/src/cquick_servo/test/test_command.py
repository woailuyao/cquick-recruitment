def test_command_format():
    # 模拟你 driver 里的拼字符串逻辑
    cmd = f"#{1:03d}P{1700:04d}T{1000:04d}!"
    assert cmd == "#001P1700T1000!"
