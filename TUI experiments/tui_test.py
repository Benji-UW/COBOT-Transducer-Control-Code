import time
import os
from time import sleep
# console.

# console.render()

i = 0
# pbar = tqdm(total=100)

t = time.time()


while i < 500:
    this_loop = time.time()
    a = i**2 - 6*i -5000
    os.system('cls||clear')
    output = ""

    output += f"According to my calculus, a = {a}\n"
    output +="That's interesting.\n"
    output +=f"i itself is actually {i} right now\n"
    output +="foo\n"
    output +=f"f-strings can do math too! {i**2}\n"
    output +="baz\n"
    output +="bar\n"
    output +=f"Time elapsed: {time.time() - t}\n"
    d = time.time() - this_loop
    output +=f"Length of this loop: {d}\n"

    print(output)
    sleep(0.05 - d)
    i += 1

print(f"Total time to run through everything: {time.time() - t} seconds")

