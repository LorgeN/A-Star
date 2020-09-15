import queue


class Search:
    """Class wrapper for A* search algorithm instance

    Contains logic for executing an A* pathfinding search on
    the given course. Considers arc cost based on course map
    values.
    """

    def __init__(self, course):
        """
        Creates a new instance of the A* search
        :param course: The course to find a path on, containing
        a start and goal. CourseMap object.
        """

        self.course = course

        # This could also be a list, since we know the size of our environment. However,
        # if we are working at an offset environment (e. g. coords starting at 100000) we
        # would waste a lot of memory, which could be a problem
        self.nodes = dict()

        # Define the initial state
        start_pos = self.course.get_start_pos()
        self._initial_state = State(start_pos[0], start_pos[1])

        # Use a PriorityQueue since it has optimizations for the case we have here. This
        # will be faster than using a list, where we would have to add to the list, and then
        # run sort (on an already mostly sorted list)
        self.open_nodes = queue.PriorityQueue()

    @property
    def closed_nodes(self):
        """
        :return: All nodes that are found and closed
        """
        # Instead of maintaining another list for this, since we do not
        # have any explicit need for it, we just compute the list if it
        # is required
        return [node for node in self.nodes.values() if not node.open]

    @property
    def initial_state(self):
        """
        :return: The initial starting point state for the path
        """
        return self._initial_state

    @property
    def goal_state(self):
        """
        :return: The goal state for the path, where we want to end up
        after travelling along it
        """
        goal_pos = self.course.get_goal_pos()
        return State(goal_pos[0], goal_pos[1])

    def find_path(self):
        """
        Finds the path on the course to go from the initial state to the
        goal state
        :return: The path. If no path is found, None is returned
        """

        final_node = self.best_first_search()
        # No path found
        if final_node is None:
            return None

        path = []
        # Iterate through each parent and add to the list
        while final_node is not None:
            path.append(final_node.state)
            final_node = final_node.parent

        return path

    def best_first_search(self):
        """
        Finds the final node in the path
        :return: The final node. If no path is found, returns None
        """

        # Create the initial node, and add it to the list of open
        # nodes for consideration
        n0 = Node(self._initial_state, self, None)
        self.open_nodes.put(n0)
        self.nodes[n0.hash()] = n0

        # While we have open nodes remaining, and have not found a
        # path to the goal state we keep looking
        while not self.open_nodes.empty():
            # self.open_nodes is sorted so that the node with the best
            # f value will be selected first
            x = self.open_nodes.get()
            x.open = False

            # Check if we are at the goal state
            if x.state == self.goal_state:
                return x

            # Generate all successors for the current node
            successors = self._generate_all_successors(x)

            # Check if we have found a new node, or if we have found a more
            # desirable path to an already discovered node
            for s in successors:
                s_hash = s.hash()
                exists = s_hash in self.nodes

                # Insert into hash table for all existing nodes if it doesn't
                # exist
                if not exists:
                    self.nodes[s_hash] = s
                # Or fetch existing object if it does
                else:
                    s = self.nodes[s_hash]

                # Add s as a child of x in case we need to propagate path
                # improvements later
                x.children.append(s)

                # Node didn't previously exist, nothing to compare to
                if not exists:
                    self.open_nodes.put(s)
                # Check if we found a more desirable path to s, and update
                # variables if we did
                elif (x.g + s.arc_cost) < s.g:
                    s.parent = x

                    if not s.open:
                        self._propagate_path_improvements(s)

        # No path has been found
        return None

    def _generate_all_successors(self, node):
        """
        Internal function for generating all successors (Surrounding nodes) for
        any given node. Considers if their position is valid.
        :param node: The origin node
        :return: The possible successors. Possible lengths between 0 and 4
        """
        nodes = []

        self._add_node_if_valid(node, 1, 0, nodes)
        self._add_node_if_valid(node, 0, 1, nodes)
        self._add_node_if_valid(node, -1, 0, nodes)
        self._add_node_if_valid(node, 0, -1, nodes)

        return nodes

    def _add_node_if_valid(self, node, x, y, nodes):
        """
        Internal utility function for the #_generate_all_successors() method. Checks
        if an adjacent node is relevant for succession.
        :param node: The origin node
        :param x: The offset for x coordinate
        :param y: The offset for y coordinate
        :param nodes: The list of nodes to add to if position is valid
        """
        state = node.state.copy_and_add(x, y)
        # Illegal position
        if state.get_arc_cost(self) == -1:
            return
        nodes.append(Node(state, self, node))

    def _propagate_path_improvements(self, node):
        """
        Internal function for propagating path improvements. If a new path is found to
        the given node, that may mean that this node is a better suited parent for its
        children because it now has a lower g value.
        :param node: The updated parent node
        """
        for child in node.children:
            # Check if the given node is now a more suitable parent for the child
            if (node.g + child.arc_cost) >= child.g:
                continue

            # Update parent
            child.parent = node
            # Recursively propagate path improvement
            self._propagate_path_improvements(child)


class State:
    """Represents the state of a node.

    In this case, we simply store a 2D position here, which is really
    all we need to identify the unique state we are at. This is the
    most common use case for A*, and all that is needed for this task.
    """

    def __init__(self, x, y):
        """
        Creates a new instance of the state
        :param x: The x coordinate of this state
        :param y: The y coordinate of this state
        """
        self.x = x
        self.y = y

    # Override a few methods to allow for pretty printing of lists with states,
    # mainly the path returned by the Search#find_path() method
    def __repr__(self):
        return self.__str__()

    def __str__(self) -> str:
        return f"({self.x}, {self.y})"

    def copy_and_add(self, x, y):
        """
        Creates an offset state
        :param x: The amount to add to the x coordinate
        :param y: The amount to add to the y coordinate
        :return: The newly created state
        """
        return State(self.x + x, self.y + y)

    def get_position(self):
        """
        :return: This state as a position tuple
        """
        return self.x, self.y

    def get_arc_cost(self, search):
        """
        :param search: The search we are currently conducting
        :return: The arc cost of visiting this state
        """
        return search.course.get_cell_value((self.x, self.y))

    def get_estimate_goal_cost(self, search):
        """
        Estimates the arc cost of moving from this state to the
        goal state
        :param search: The search we are currently conducting
        :return: The estimated cost
        """
        goal_state = search.goal_state
        # Simple manhattan distance estimate
        return abs(goal_state.x - self.x) + abs(goal_state.y - self.y)

    def hash(self, search):
        """
        Generates a unique hash based on this unique state
        :param search: The search we are currently conducting
        :return: The generated unique hash
        """
        return (self.x * search.course.get_height()) + self.y

    def __eq__(self, other):
        if isinstance(other, State):
            return self.get_position() == other.get_position()
        return NotImplemented


class Node:
    """Represents a node in a search.

    Contains the state of the node, if the node is open/closed, the
    best parent for the node and the node's children
    """

    def __init__(self, state, search, parent):
        self.state = state
        self.search = search
        self.open = True
        self.parent = parent
        self.children = []

    def hash(self):
        """
        :return: The hash of this node's state
        """
        return self.state.hash(self.search)

    @property
    def arc_cost(self):
        """
        :return: The arc cost of this node's state
        """
        return self.state.get_arc_cost(self.search)

    @property
    def g(self):
        """
        :return: The g value of this node, per A* definition
        """
        self_cost = self.arc_cost
        # Check if we are in the inital node, in which case we have no
        # parent
        if self.parent is None:
            return self_cost
        return self.parent.g + self_cost

    @property
    def h(self):
        """
        :return: The h value of this node, per A* definition
        """
        return self.state.get_estimate_goal_cost(self.search)

    @property
    def f(self):
        """
        :return: The f value of this node, per A* definition
        """
        return self.g + self.h

    # Functions to make this object comparable so we can use
    # the PriorityQueue for better performance. They are comparable
    # so that the one with the lowest f value will be placed first

    def __lt__(self, obj):
        return self.f < obj.f

    def __le__(self, obj):
        return self.f <= obj.f

    def __eq__(self, obj):
        return self.f == obj.f

    def __ne__(self, obj):
        return self.f != obj.f

    def __gt__(self, obj):
        return self.f > obj.f

    def __ge__(self, obj):
        return self.f >= obj.f
