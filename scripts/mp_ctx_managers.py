import multiprocessing as mp
from multiprocessing.managers import BaseManager

class CustomObject:
    def __init__(self, value=0):
        self.value = value

    def increment(self):
        self.value += 1

    def get_value(self):
        return self.value

class CustomManager(BaseManager):
    pass

CustomManager.register('CustomObject', CustomObject) # I see. If you do not register the class, you cannot use it with the manager.


def main():
    # Get the context from the current thread
    context = mp.get_context()

    # Create a CustomManager with the context
    manager = CustomManager(ctx=context)
    manager.start()

    # Create an instance of CustomObject using the CustomManager
    custom_object_proxy = manager.CustomObject(value=5)

    # Perform operations on the custom object
    custom_object_proxy.increment()
    print("Value of custom object:", custom_object_proxy.get_value())

    manager.shutdown()

if __name__ == '__main__':
    main()
