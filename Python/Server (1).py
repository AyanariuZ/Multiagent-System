import mesa

from model import Environment, Bot, Box, Exit, Caja

MAX_NUMBER_ROBOTS = 20


def agent_portrayal(agent):
    if isinstance(agent, Bot):
        return {"Shape": "circle", "Filled": "false", "Color": "Cyan", "Layer": 1, "r": 0.9,
                "text": f"{agent.battery}", "text_color": "black"}
    elif isinstance(agent, Box):
        return {"Shape": "rect", "Filled": "true", "Layer": 0, "w": 0.9, "h": 0.9, "text_color": "Black",
                "Color": "#ccbeaf", "text": "📦"}
    elif isinstance(agent, Exit):
        return {"Shape": "rect", "Filled": "true", "Layer": 0, "w": 0.9, "h": 0.9, "text_color": "Blue",
                "Color": "#ccbeaf", "text": "E"}
    elif isinstance(agent, Caja):
        return {"Shape": "rect", "Filled": "true", "Layer": 0, "w": 0.9, "h": 0.9, "text_color": "Brown",
                "Color": "#ccbeaf", "text": "M"}
    else:
        return {"Shape": "rect", "Filled": "true", "Layer": 0, "w": 0.9, "h": 0.9, "text_color": "Black",
                "Color": "white", "text": ""}
    


grid = mesa.visualization.CanvasGrid(
    agent_portrayal, 20, 20, 400, 400)

# Create a chart to track the battery of the robots
chart_charges = mesa.visualization.ChartModule(
    [
        {"Label": "MinBattery", "Color": "#DD3B30", "label": "Min Battery"},
        {"Label": "MaxBattery", "Color": "#403EDD", "label": "Max Battery"},
        {"Label": "MeanBattery", "Color": "#DD1BD7", "label": "Mean Battery"},
    ],
    data_collector_name='datacollector'
)

model_params = {
    "num_agents": mesa.visualization.Slider(
        "Number of Robots",
        5,
        2,
        MAX_NUMBER_ROBOTS,
        1,
        description="Choose how many robots to include in the model",
    ),
    "obstacle_portion": mesa.visualization.Slider(
        "Obstacle Portion",
        0.3,
        0.0,
        0.75,
        0.05,
        description="Choose the percentage of obstacles in the environment",
    ),
    "mode_start_pos": mesa.visualization.Choice(
        "Bot Start Position Mode",
        "Random",
        ["Fixed", "Random"],
        "Choose whether to place the robots in a fixed position or randomly",
    ),
    "M": 20,
    "N": 20,
}

server = mesa.visualization.ModularServer(
    Environment, [grid, chart_charges],
    "Example1", model_params, 8521
)

server.launch(open_browser=True)
