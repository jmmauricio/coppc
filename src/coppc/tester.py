def save_stream_to_file(filename, data_stream):
    """
    Save data from a continuous stream to a file as it arrives.
    
    Parameters:
    - filename: str, name of the file to write to.
    - data_stream: generator or iterable that yields data.
    """
    try:
        with open(filename, 'a') as file:
            for data in data_stream:
                file.write(data + '\n')  # Append data to file
                print(f"Appended '{data}' to {filename}")
    
    except Exception as e:
        print(f"Error: {e}")

# Example: Simulating a continuous data stream
import time
import random

def simulated_data_stream():
    """Simulate a data stream by yielding random data."""
    while True:
        yield f"Data-{random.randint(1, 100)}"
        time.sleep(2)  # Simulate data arriving every 2 seconds

# Call the function with simulated data stream
save_stream_to_file('stream_output.txt', simulated_data_stream())
