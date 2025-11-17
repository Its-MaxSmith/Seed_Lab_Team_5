# lcd_updater
#
# Max Smith
# 11/13/2025

# Imports
import LCD_print_op
import threading
import time

lcd_msg = "No marker found"
lcd_lock = threading.Lock()

class LCDupdater:

    def __init__(self, update_interval = 1.0):
        self.update_interval = update_interval  # seconds
        self.msg = "No marker found"
        self.lock = threading.Lock()
        self._running = False
        self.thread = None

    def start(self):
        # Start the background LCD
        if self._running:
            return  # already running
        self._running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def stop(self):
        # Stop the background LCD
        self._running = False
        if self.thread is not None:
            self.thread.join(timeout=1)

    def update_message(self, new_msg):
        # Update the message
        with self.lock:
            self.msg = new_msg

    def _run(self):
        # Internal loop that updates the LCD
        while self._running:
            with self.lock:
                msg_copy = self.msg
            try:
                LCD_print_op.LCD_print(msg_copy)
            except Exception as e:
                print(f"LCD error: {e}")
            time.sleep(self.update_interval)


