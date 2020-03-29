from __future__ import print_function
import pandas as pd
import pdb
import logging
import os
import time as systime
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
logger = logging.getLogger(__name__)
'''Documetation of ORtools:https://developers.google.com/optimization/routing
'''
class Get_Inputs:
    def __init__(self,input_dir):
        if not os.path.exists(input_dir):
            logging.error('File Diretory does not exist')
            raise Exception(input_dir + ' does not exist!')
        self.data = {}
        self.data['depot'] = 0 #which node the vehicle starts
        self.data['dist_matrix'] = self.read_data_file(os.path.join(input_dir, 'dist_matrix.csv'))
        self.data['time_matrix'] = self.read_data_file(os.path.join(input_dir, 'time_matrix.csv'))
        self.data['time_windows'] = self.read_data_file(os.path.join(input_dir, 'time_constraint.csv'))
        print('Done with Reading Data!')

    def read_data_file(self,file_path):
        try:
            with open(file_path,'r') as f:
                input_data = []
                for line in f:
                    input_data.append([int(num) for num in line.split(',')])
            logger.info('Read '+file_path)
        except:
            logging.error(file_path + ' does not exist!')
            print(file_path + ' does not exist!')
        #return np.array(input_data, dtype=float)
        return input_data

class CP_localsearch:
    def __init__(self, input_data, VisitTime=0, num_vehicles = 1, output_dir='Results'):
        self.data= input_data.data
        self.data['num_vehicles'] = num_vehicles
        self.VisitTime = VisitTime*60
        self.sol = {}
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
        self.output_dir = output_dir

    def solve_model(self,max_wait_time = 0,max_per_veh = 99999, save_output=True):
        """Solve the VRP with time windows."""
        self.manager = pywrapcp.RoutingIndexManager(len(self.data['time_matrix']),
                                               self.data['num_vehicles'], self.data['depot'])
        routing = pywrapcp.RoutingModel(self.manager)

        transit_callback_index = routing.RegisterTransitCallback(self.time_callback)
        routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
        time = 'Time'
        routing.AddDimension(
            transit_callback_index,
            max_wait_time,  # allow waiting time
            max_per_veh,  # maximum time per vehicle
            False,  # Don't force start cumul to zero.
            time)
        time_dimension = routing.GetDimensionOrDie(time)
        # Add time window constraints for each location except depot.
        for location_idx, time_window in enumerate(self.data['time_windows']):
            if location_idx == 0:
                continue
            index = self.manager.NodeToIndex(location_idx)
            time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1]-self.VisitTime)
        # Add time window constraints for each vehicle start node.
        for vehicle_id in range(self.data['num_vehicles']):
            index = routing.Start(vehicle_id)
            time_dimension.CumulVar(index).SetRange(self.data['time_windows'][self.data['depot']][0],
                                                    self.data['time_windows'][self.data['depot']][1])
        for i in range(self.data['num_vehicles']):
            routing.AddVariableMinimizedByFinalizer(
                time_dimension.CumulVar(routing.Start(i)))
            routing.AddVariableMinimizedByFinalizer(
                time_dimension.CumulVar(routing.End(i)))
        '''Routing Settings:https://developers.google.com/optimization/routing/routing_options
        '''
        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.PATH_MOST_CONSTRAINED_ARC)
        search_parameters.time_limit.seconds = 3600
        #search_parameters.log_search = True
        search_parameters.local_search_metaheuristic = (
            routing_enums_pb2.LocalSearchMetaheuristic.AUTOMATIC)
        sol_status={0:'ROUTING_NOT_SOLVED: Problem not solved yet.',
                    1:'ROUTING_SUCCESS: Problem solved successfully.',
                    2:'ROUTING_FAIL: No solution found to the problem.',
                    3:'ROUTING_FAIL_TIMEOUT: Time limit reached before finding a solution.',
                    4:'ROUTING_INVALID: Model, model parameters, or flags are not valid.'}
        print('Start Solving the problem....')
        _start_ = systime.time()
        assignment = routing.SolveWithParameters(search_parameters)
        print("Solver status: ", sol_status[routing.status()])
        soltime = systime.time()-_start_
        print('Solving takes: '+ str(round(soltime,2))+' Secs')
        if assignment:
            self.print_save_solution(routing, assignment,save_res=save_output)
        else:
            print('Solving Failed')
    def time_callback(self, from_index, to_index):
        """Returns the travel time between the two nodes."""
        # Convert from routing variable Index to time matrix NodeIndex.
        from_node = self.manager.IndexToNode(from_index)
        to_node = self.manager.IndexToNode(to_index)
        return self.data['time_matrix'][from_node][to_node]
    def print_save_solution(self,routing, assignment,save_res=True):
        """Prints assignment on console."""
        time_dimension = routing.GetDimensionOrDie('Time')
        total_time = 0
        for vehicle_id in range(self.data['num_vehicles']):
            index = routing.Start(vehicle_id)
            plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
            while not routing.IsEnd(index):
                time_var = time_dimension.CumulVar(index)
                plan_output += '{0} Time({1},{2}) -> '.format(
                    self.manager.IndexToNode(index), assignment.Min(time_var),
                    assignment.Max(time_var))
                index = assignment.Value(routing.NextVar(index))
            time_var = time_dimension.CumulVar(index)
            plan_output += '{0} Time({1},{2})\n'.format(self.manager.IndexToNode(index),
                                                        assignment.Min(time_var),
                                                        assignment.Max(time_var))
            plan_output += 'Time of the route: {} min\n'.format(
                assignment.Min(time_var))
            print(plan_output)
            total_time += assignment.Min(time_var)
        print('Total time of all routes: {}min'.format(total_time))

        if save_res:
            for vehicle_id in range(self.data['num_vehicles']):
                output = []
                index = routing.Start(vehicle_id)
                seq = 1
                time_win_end = 0
                last_node = self.manager.IndexToNode(index)
                while not routing.IsEnd(index):
                    cur_node = self.manager.IndexToNode(index)
                    dist = self.data['dist_matrix'][last_node][cur_node]
                    time_var = time_dimension.CumulVar(index)
                    drive_time = self.data['time_matrix'][last_node][cur_node]
                    time_win_start = assignment.Min(time_var)
                    time_win_end = assignment.Max(time_var)
                    output.append([seq,index,round(dist/1609.34,2),round(drive_time/60,2),time_win_start,time_win_end])
                    seq += 1
                    last_node = self.manager.IndexToNode(index)
                    index = assignment.Value(routing.NextVar(index))
                output_df = pd.DataFrame(output, columns=['Sequence','Node','Distance',
                                                          'Driving_Time', 'Arrival',
                                                          'Leave'])
                output_df.to_csv(os.path.join(self.output_dir,f'result_vehicle{vehicle_id}.csv'), index=False)
if __name__ == '__main__':
    Data = Get_Inputs('data/')
    TSPTW = CP_localsearch(Data,VisitTime=30,num_vehicles=1)
    TSPTW.solve_model()