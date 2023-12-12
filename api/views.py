import json
import os
import random
from . import config
from django.views.decorators.csrf import csrf_exempt
from django.http import HttpResponse, JsonResponse
from rest_framework.response import Response

@csrf_exempt
def agent_path(request):
    if request.method == 'POST':
        try:
            data = json.loads(request.body)
            selectedMap = data.get('mapName')
            selectdAgent = data.get('characterId')

            map = [1, 4, 3, 2, 0]

            response_data = {
                'updatedAgentPath': map,
                'selectedCharacterId': selectedMap,
                'mapName': selectdAgent,
            }
            return JsonResponse(response_data)
        except Exception as e:
            print(e)
            return HttpResponse(status=400)
    else:
        return HttpResponse(status=405) 
    
def generate_cost_matrix(map_data, numRows):
    cost_matrix = [[0] * numRows for _ in range(numRows)]

    for i in range(numRows):
        values = map_data[i][2:]
        for j, value in enumerate(values):
            rowIndex, colIndex = i, j
            if colIndex < numRows:
                cost_matrix[rowIndex][colIndex] = value
                cost_matrix[colIndex][rowIndex] = value

    return cost_matrix

def get_updated_agent_path(numRows):
        return random.sample(range(1, numRows), numRows-1) + [0]

def load_map(map_name):
    map_file_path = os.path.join(config.MAP_FOLDER, f'{map_name}.txt')

    if os.path.exists(map_file_path):
        with open(map_file_path, 'r') as file:
            map_data = [list(map(int, line.strip().split(','))) for line in file]
        return map_data
    return None
