import asyncio

async def process1():
    # Asynchronous code for process 1
    await asyncio.sleep(2)  # Simulating some asynchronous task
    return "Result 1"

async def process2():
    # Asynchronous code for process 2
    await asyncio.sleep(3)  # Simulating some asynchronous task
    return "Result 2"

async def main():
    # Create tasks for each process
    task1 = asyncio.create_task(process1())
    task2 = asyncio.create_task(process2())

    # Wait for both tasks to complete
    results = await asyncio.gather(task1, task2)

    # Print the results
    for result in results:
        print(result)

# Run the main coroutine
asyncio.run(main())
