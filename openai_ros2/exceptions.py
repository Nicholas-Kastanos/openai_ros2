class SpinFutureTimeoutException(Exception):
    """Raised when the spin_until_future_complete operation times out."""

    def __init__(self, *args):
        Exception.__init__(self, 'spin_until_future_complete() timed out', *args)