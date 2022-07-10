import rich
from tqdm import tqdm
import time
from rich.console import Console
from rich.progress import track

console = Console()


console.print("Wee I made a terminal")
# console.

# console.render()

i = 0
# pbar = tqdm(total=100)

def update_console():
    pbar.update()

while i < 100:
    # update_console()
    console.clear()
    pbar = tqdm(total=100, initial=i)
    time.sleep(0.05)
    i += 1
