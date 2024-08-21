from mesa.model import Model
from mesa.agent import Agent
from mesa.space import SingleGrid
from mesa.time import SimultaneousActivation
from mesa.datacollection import DataCollector

import numpy as np


class Box(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)

class Caja(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)

class Exit(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)

class CentralSystem(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)
        self.tasks = []  # Lista de tareas pendientes
        self.robot_statuses = {}  # Estado de cada robot (Libre/Ocupado)
        self.battery = 34;

    def step(self):
        # Asignar tareas a los robots libres
        for bot in self.model.schedule.agents:
            if isinstance(bot, Bot):
                if self.tasks:
                    task = self.tasks.pop(0)
                    bot.receive_task(task)
                    #self.robot_statuses[robot.unique_id] = "Busy"

    def add_task(self, task):
        self.tasks.append(task)

    def report_completion(self, robot_id):
        self.robot_statuses[robot_id] = "Free"

class Bot(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)
        self.next_pos = None
        self.luck = np.random.uniform(0.2, 1.0)
        self.movements = 0
        self.battery = 1000 #np.random.randint(75, 100)
        self.path = []
        
        self.carry = False
        self.tasks = []
        self.goal = None;
        
    def receive_task(self, task):
        self.tasks.append(task)
        
        

    def manhattan_heuristic(self, pos, goal):
        return abs(pos[0] - goal[0]) + abs(pos[1] - goal[1])
    
    def at_box(self):
        # Implementar lógica para verificar si se alcanzó el objetivo
        return False
    
    def at_exit(self):
        # Implementar lógica para verificar si se alcanzó el objetivo
        return True;

    def find_exit(self):
        # Encontrar la posición de la salida ('S') en el grid
        for pos in self.model.grid.coord_iter():
            _, (x, y) = pos
            contents = self.model.grid.get_cell_list_contents([x, y])
            for obj in contents:
                if isinstance(obj, Exit):  # Exit es el tipo de agente que representa la salida
                    return (x, y)

    def a_star(self, start, goal):
        # Listas de nodos abiertos y cerrados
        open_list = []
        closed_list = set()

        # Añadir el nodo inicial a la lista abierta
        open_list.append((0 + self.manhattan_heuristic(start, goal), 0, start, None))

        while open_list:
            # Ordenar la lista abierta por f = g + h y seleccionar el nodo con menor f
            open_list.sort(key=lambda x: x[0])
            f, g, current, parent = open_list.pop(0)

            # Si alcanzamos el objetivo, reconstruimos el camino
            if current == goal:
                path = []
                while parent is not None:
                    path.append(current)
                    current = parent
                    parent = next((p for f, g, c, p in closed_list if c == current), None)
                path.reverse()
                return path

            # Añadir el nodo actual a la lista cerrada
            closed_list.add((f, g, current, parent))

            # Iterar sobre vecinos
            for neighbor in self.model.grid.iter_neighborhood(current, moore=False, include_center=False):
                if self.model.grid.is_cell_empty(neighbor) or neighbor == goal:
                    g_new = g + 1  # Distancia desde el inicio hasta el vecino
                    h_new = self.manhattan_heuristic(neighbor, goal)
                    f_new = g_new + h_new

                    # Si el vecino ya está en la lista cerrada, saltarlo
                    if any(neighbor == c for f, g, c, p in closed_list):
                        continue

                    # Si el vecino ya está en la lista abierta con un f mayor, actualizarlo
                    existing_node = next((i for i, (f, g, c, p) in enumerate(open_list) if c == neighbor), None)
                    if existing_node is not None:
                        if open_list[existing_node][0] > f_new:
                            open_list[existing_node] = (f_new, g_new, neighbor, current)
                    else:
                        open_list.append((f_new, g_new, neighbor, current))

        return []

    def step(self):
        if self.battery > 0 and self.path and self.goal:
            self.next_pos = self.path.pop(0)
            self.movements += 1
            self.battery -= 1
            if self.carry == False:
                box_in_next_pos = [agent for agent in self.model.grid.get_cell_list_contents([self.next_pos]) if isinstance(agent, Caja)]
                if box_in_next_pos:
                # Elimina la caja
                    box = box_in_next_pos[0]
                    self.model.grid.remove_agent(box)
                    self.carry = True;
                    #self.model.schedule.remove(box)
                
                    # Actualiza la meta a la salida (asumiendo que 'S' es la salida)
                    self.goal = self.find_exit()
                
                    # Vuelve a calcular el camino usando A*
                    self.path = self.a_star(self.pos, self.goal)

                self.model.grid.move_agent(self, self.next_pos)

            if self.carry == True:
                exit_in_next_pos = [agent for agent in self.model.grid.get_cell_list_contents([self.next_pos]) if isinstance(agent, Exit)]
                if exit_in_next_pos:
                # Elimina la caja
                    self.carry = False;
                    #self.model.schedule.remove(box)
                
                    # Vuelve a calcular el camino usando A*
                    self.goal = self.tasks.pop()[0]
                    self.path = self.a_star(self.pos, self.goal)
                    if self.path:
                        self.next_pos = self.path.pop(0)
                        self.movements += 1
                        self.battery -= 1
                        self.model.grid.move_agent(self, self.next_pos)
                else:
                    self.model.grid.move_agent(self, self.next_pos)
                

        elif self.battery > 0 and self.goal==None and not self.path:
            # Si no hay un camino predefinido, calcular la ruta
            self.goal = self.tasks.pop()[0]
            self.path = self.a_star(self.pos, self.goal)
            if self.path:
                self.next_pos = self.path.pop(0)
                self.movements += 1
                self.battery -= 1
                self.model.grid.move_agent(self, self.next_pos)

            
        #if self.at_target():
                #self.model.central_system.report_completion(self.unique_id)
                #self.task = None

    


class Environment(Model):
    def __init__(self, M: int, N: int,
                 num_agents: int = 5,
                 obstacle_portion: float = 0.6,
                 mode_start_pos: str = 'Fixed',
                 ):
        super().__init__()

        self.num_agents = num_agents
        self.obstacle_portion = obstacle_portion
        self.grid = SingleGrid(M, N, False)
        self.schedule = SimultaneousActivation(self)
        self.special_cell = []

        self.central_system = CentralSystem(0, self)
        self.schedule.add(self.central_system)

        # Environment setup
        #self._automatic_placement(M, N, num_agents, obstacle_portion, mode_start_pos)

        # Example of manual placement for a 20x20 grid, maze-like environment
        desc = ['BBBBBBBBBBBBBBBBBBBB',
                'BMMMMMMMFFFFFFFFFFFB',
                'BFFFFFFFFFFFFFFFFFFB',
                'BFFFFFFFFFFFFFFFFFFB',
                'BFFFFFFFFFFFFFFFFFFB',
                'BFFFFFFFFFFFFFFFFFFB',
                'BFFFFFFFFFFFFFFFFFFB',
                'BFFFFFFFFFFFFFFFFFFB',
                'BFFFFFFFFFFFFFFFFFFB',
                'BFRFFFFFFFBBBBBBBBBB',
                'BFFFFFFFFFFFFFFFFFFB',
                'BFFFFFFFFFFFFFFFFFFB',
                'BFFFFFFFFFFFFFFFFFFB',
                'BFFFFFFFFFFFFFFFFFFB',
                'BFFFFFFFFFFFFFFFFFFB',
                'BBBBBBBBBBBFFFFFFFFB',
                'BFFFFFFFFFFFFFFFFFFB',
                'BFFFFFFFFFFFFFFFFFFB',
                'BFFFFFFFFFFFFFFFFFSB',
                'BBBBBBBBBBBBBBBBBBBB']

        self._manual_placement(desc)

        # Data collector
        self.datacollector = DataCollector(
            model_reporters={
                "MinBattery": lambda m: np.min(
                    [a.battery for a in m.schedule.agents]).astype(float),
                "MaxBattery": lambda m: np.max(
                    [a.battery for a in m.schedule.agents]).astype(float),
                "MeanBattery": lambda m: np.mean(
                    [a.battery for a in m.schedule.agents]).astype(float),
            }
        )

        
    # Method for placing agents in the environment based on a list of positions as characters where 'B' is a box,
    # 'R' is a robot, and 'F' is a free cell. For example for a 5x5 grid: the first row is 'BFFFF', the second row is
    # 'BFFFF', the third row is 'BFFFF', the fourth row is 'BFFFF', and the fifth row is 'BFFFF'.
    def _manual_placement(self, desc: list):

        box_positions = []
        goal_position = (0,0)
        
        M, N = self.grid.height, self.grid.width

        for pos in self.grid.coord_iter():
            _, (x, y) = pos
            if desc[M - y - 1][x] == 'B':
                box = Box(int(f"{x}{y}") + 1, self)
                self.grid.place_agent(box, (x, y))
                
            elif desc[M - y - 1][x] == 'M':
                goal = (x, y)
                caja = Caja(int(f"{x}{y}") + 1, self)
                self.special_cell.append(goal)
                box_positions.append((x, y))
                self.grid.place_agent(caja, (x, y))

            elif desc[M - y - 1][x] == 'R':
                bot = Bot(int(f"{x}{y}") + 1, self)
                self.grid.place_agent(bot, (x, y))
                self.schedule.add(bot)
            elif desc[M - y - 1][x] == 'S':
                exitt = Exit(int(f"{x}{y}") + 1, self)
                self.grid.place_agent(exitt, (x, y))
                goal_position = (x, y)

        for box_pos in box_positions:
            self.central_system.add_task((box_pos, goal_position))
            
            

    def step(self):
        self.datacollector.collect(self)

        self.schedule.step()

        self.running = any([a.battery > 0 for a in self.schedule.agents])
