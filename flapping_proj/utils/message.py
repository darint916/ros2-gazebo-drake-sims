class Message:
    class Colors:
        RESET = '\033[0m'
        BOLD = '\033[1m'
        UNDERLINE = '\033[4m'
        BLUE = '\033[94m'
        GREEN = '\033[92m'
        YELLOW = '\033[93m'
        RED = '\033[91m'
        CYAN = '\033[96m'
        WHITE = '\033[97m'
        MAGENTA = '\033[95m'
        GREY = '\033[90m'
        LIGHT_GREEN = '\033[92m'
        LIGHT_BLUE = '\033[94m'
        LIGHT_YELLOW = '\033[93m'
        LIGHT_RED = '\033[91m'
        LIGHT_CYAN = '\033[96m'

    @staticmethod
    def print_formatted(msg, *styles):
        style_format = ''.join(styles)
        print(f"{style_format}{msg}{Message.Colors.RESET}")

    @staticmethod
    def info(msg, bold=False):
        styles = [Message.Colors.BLUE]
        if bold:
            styles.append(Message.Colors.BOLD)
        Message.print_formatted(msg, *styles)

    @staticmethod
    def warning(msg, bold=False):
        styles = [Message.Colors.YELLOW]
        if bold:
            styles.append(Message.Colors.BOLD)
        Message.print_formatted(msg, *styles)

    @staticmethod
    def error(msg, bold=True):
        styles = [Message.Colors.RED]
        if bold:
            styles.append(Message.Colors.BOLD)
        Message.print_formatted(msg, *styles)

    @staticmethod
    def success(msg, bold=False):
        styles = [Message.Colors.GREEN]
        if bold:
            styles.append(Message.Colors.BOLD)
        Message.print_formatted(msg, *styles)

    @staticmethod
    def debug(msg, bold=False):
        styles = [Message.Colors.CYAN]
        if bold:
            styles.append(Message.Colors.BOLD)
        Message.print_formatted(msg, *styles)

    @staticmethod
    def custom(msg, color=None, bold=False, underline=False):
        styles = []
        if color:
            styles.append(getattr(Message.Colors, color.upper(), ''))
        if bold:
            styles.append(Message.Colors.BOLD)
        if underline:
            styles.append(Message.Colors.UNDERLINE)
        Message.print_formatted(msg, *styles)

    @staticmethod
    def data(msg, bold=False):
        styles = [Message.Colors.MAGENTA]
        if bold:
            styles.append(Message.Colors.BOLD)
        Message.print_formatted(msg, *styles)
# Usage
# Message.info("This is an informational message")
# Message.warning("Warning: Consider this a warning!", bold=True)
# Message.error("Error: Something went wrong!")
# Message.success("Success: Task completed successfully!", bold=True)
# Message.debug("Debug: Here's a debug message")