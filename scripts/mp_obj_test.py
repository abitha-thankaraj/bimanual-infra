# import multiprocessing as mp

# class CustomObject:
#     def __init__(self):
#         self.value = 0

#     def increment(self):
#         self.value += 1

#     def get_value(self):
#         return self.value

# def increment_object(custom_object):
#     for _ in range(10):
#         print("Adding 1 to custom object...")
#         custom_object.increment()

# if __name__ == '__main__':
#     # Create a Manager
#     manager = mp.Manager()

#     # Wrap the CustomObject using the Manager
#     custom_object = manager.Namespace()
#     custom_object.data = CustomObject()

#     # Create multiple processes that increment the custom_object's value
#     processes = [mp.Process(target=increment_object, args=(custom_object.data,)) for _ in range(5)]

#     # Start the processes
#     for process in processes:
#         process.start()

#     # Wait for the processes to finish
#     for process in processes:
#         process.join()

#     # Print the final value of the custom object
#     print("Final value of custom object:", custom_object.data.get_value())

import multiprocessing as mp
from multiprocessing.managers import BaseManager

class CustomObject:
    def __init__(self):
        self.value = 0

    def increment(self):
        self.value += 1

    def get_value(self):
        return self.value

class CustomManager(BaseManager):
    pass

CustomManager.register('CustomObject', CustomObject)

def increment_object(custom_object_proxy):
    for _ in range(10):
        custom_object_proxy.increment()

if __name__ == '__main__':
    # Create a CustomManager
    manager = CustomManager()
    manager.start()

    # Create an instance of CustomObject using the CustomManager
    custom_object_proxy = manager.CustomObject()

    # Create multiple processes that increment the custom_object_proxy's value
    processes = [mp.Process(target=increment_object, args=(custom_object_proxy,)) for _ in range(5)]

    # Start the processes
    for process in processes:
        process.start()

    # Wait for the processes to finish
    for process in processes:
        process.join()

    # Print the final value of the custom object
    print("Final value of custom object:", custom_object_proxy.get_value())
