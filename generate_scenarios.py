import itertools
import pandas as pd

def count_inversions(sequence):
    inversions = 0
    for i in range(len(sequence)):
        for j in range(i + 1, len(sequence)):
            if sequence[i] > sequence[j] and sequence[i] != 0 and sequence[j] != 0:
                inversions += 1
    return inversions

def is_solvable(permutation):
    """
    For a 3x3 grid, if the number of inversions is odd, the puzzle is unsolvable
    therefore, we are going to count the number of inversion (inversion = if 2 successive numbers are in reverse 
    order and none of them is 0 (*))
    then, check if it is even or odd
        if odd, it is solvable
        else, it is not
    """
    return count_inversions(permutation) % 2 == 0

numbers = list(range(9))
permutations_list = list(itertools.permutations(numbers))

# Filter out unsolvable permutations
solvable_permutations = [p for p in permutations_list if is_solvable(p)]

# Convert to DataFrame
Converted_data = pd.DataFrame(solvable_permutations)

# Write to CSV
filename = "scenarios.csv"
Converted_data.to_csv(filename, index=False, header=False)
