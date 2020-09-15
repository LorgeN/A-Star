from path import *
from map import *
import time


def get_time():
    return int(round(time.time() * 1000))


# Entry point for using A* algorithm implementation to complete
# assignment pathfinding tasks

# Accept an input for which task we are running. Will error if
# someone enters something that isn't an integer, but that isn't
# really something we care very much about here
task = int(input("Please enter a task (number) to run: "))

print("Loading map...")
course = CourseMap(task=task)
print("Map loaded.")

print("Initializing search...")
search = Search(course)
print("Finding path...")

# Track the amount of time in ms it takes for the algorithm runs
start = get_time()
path = search.find_path()
end = get_time()

print("Path:", path)
print(f"Took {end - start}ms")

str_map = course.str_map.copy()

# Apply the path to the copy of the str_map
for point in path:
    pos = point.get_position()
    val = str_map[pos[0]][pos[1]]
    if val == ' S ' or val == ' G ':
        continue

    str_map[pos[0]][pos[1]] = -2

course.show_map(map=str_map)
