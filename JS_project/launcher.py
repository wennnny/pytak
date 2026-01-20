import subprocess
import threading
import sys
from waypoint_sync_gui import App  # 直接引用你的 GUI 類別

def start_main_program():
    """
    在背景執行 main.py
    """
    subprocess.Popen([sys.executable, "main_ui.py"])

if __name__ == "__main__":
    # 啟動 GUI
    app = App()

    # 修改 GUI 的 start 按鈕，使它也能啟動 main.py
    old_start = app.on_start

    def new_start():
        # 先執行 GUI 自己的 on_start（啟動 CoT→UDP 工具）
        old_start()

        # 再啟動 main.py
        threading.Thread(target=start_main_program, daemon=True).start()

    app.on_start = new_start

    app.mainloop()
