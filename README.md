# Python A* Implementation
This implementation is written as part of assignment 2 for the Introduction to AI (TDT4136) class at NTNU. The project finds a path between two given static points in a 2D world. Supports different arc costs. Uses Manhattan distance for heuristics.

## Executing code
(Linux/macOS)
```bash
$ python3 -m venv env
$ source env/bin/activate
$ pip install -r requirements.txt
$ python3 main.py
``` 
If you are using Windows execute `.\env\Scripts\activate` instead of the `source` command

This will prompt for a task number. Supported values are integers between 1 and 4.