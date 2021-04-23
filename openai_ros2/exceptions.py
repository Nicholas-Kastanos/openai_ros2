class SpinFutureTimeoutException(Exception):
    """Raised when the spin_until_future_complete operation times out."""

    def __init__(self, *args):
        Exception.__init__(self, 'spin_until_future_complete() timed out', *args)

class NamespaceNotSetException(Exception):
    """Raised when the ControllerConnector is used without a valid namespace."""

    def __init__(self, *args):
        Exception.__init__(self, 'The robot namespace must be set to use this method', *args)