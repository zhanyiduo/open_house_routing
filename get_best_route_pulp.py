import pandas as pd
import pdb
import cvxpy as cp
import pulp as pl
import logging
import numpy as np
import os
import itertools
logger = logging.getLogger(__name__)

class Get_Inputs:
    def __init__(self,input_dir):
        if not os.path.exists(input_dir):
            logging.error('File Diretory does not exist')
            raise Exception(input_dir + ' does not exist!')
        self.dist = self.read_data_file(os.path.join(input_dir,'dist_matrix.csv'))
        self.time = self.read_data_file(os.path.join(input_dir, 'time_matrix.csv'))
        self.timelim = self.read_data_file(os.path.join(input_dir, 'time_constraint.csv'))
        print('Done with Reading Data!')

    def read_data_file(self,file_path):
        try:
            with open(file_path,'r') as f:
                input_data = [[num for num in line.split(',')] for line in f]
            logger.info('Read '+file_path)
        except:
            logging.error(file_path + ' does not exist!')
            print(file_path + ' does not exist!')
        return np.array(input_data, dtype=float)

class Traditional_Optimizer:
    def __init__(self,input_data,weights,VisitTime=10,output_dir=None,time_constraint=True):
        self.data = input_data
        self.time_constraints = time_constraint
        self.Nloc = input_data.dist.shape[0]
        self.wts = weights
        self.dim = range(self.Nloc)
        self.SourceIndex = 0
        self.VisitTime = VisitTime
        self.bigM = 1e+10
        self.timelimits = 20*60
        self.sol = dict()
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
        self.output_dir = os.path.join(output_dir,'results.csv')

    def get_shortest_route(self):
        mdl = pl.LpProblem('VRPTW',pl.LpMinimize)
        variables = {}
        variables['x'] = pl.LpVariable.dicts('x', (self.dim,self.dim),
                                  lowBound=0,
                                  upBound=1,
                                  cat=pl.LpBinary)#the routing decision between location i and j
        variables['u'] = pl.LpVariable.dicts('u', self.dim,
                                lowBound=1,
                                upBound=None,
                                cat=pl.LpInteger)#step u at location j
        variables['t'] = pl.LpVariable.dicts('t', self.dim,
                                lowBound=0,
                                upBound=None,
                                cat=pl.LpContinuous)  # arrival time at step k
        variables['ArriveEarly'] = pl.LpVariable.dicts('ArriveEarly', self.dim,
                                lowBound=0,
                                upBound=None,
                                cat=pl.LpContinuous)  # at step k early for x miniutes
        variables['ArriveLate'] = pl.LpVariable.dicts('ArriveLate', self.dim,
                                          lowBound=0,
                                          upBound=None,
                                          cat=pl.LpContinuous)  # at step k late for x miniutes
        #create objectives
        mdl = self.add_obj(mdl,variables)
        mdl = self.add_routing_constraints(mdl,variables)
        mdl = self.add_time_constraints(mdl, variables)

        logging.info('Solving VRPTW')
        mdl.solve(pl.PULP_CBC_CMD(msg=1,maxSeconds=self.timelimits))
        status = pl.LpStatus[mdl.status]
        print('Problem Status: %s' % status)
        if status not in ['Optimal', 'Not Solved']:
            raise RuntimeError('MIP not solved properly. Current status is %s' % status)
        print('Solving Done')

        if status == 'Optimal':
            self.sol['u'] = np.zeros(self.Nloc)
            self.sol['x'] = np.zeros((self.Nloc,self.Nloc))
            self.sol['t'] = np.zeros(self.Nloc)
            self.sol['ArriveEarly'] = np.zeros(self.Nloc)
            self.sol['ArriveLate'] = np.zeros(self.Nloc)
            for i in self.dim:
                self.sol['u'][i] = variables['u'][i].varValue
                self.sol['t'][i] = variables['t'][i].varValue
                self.sol['ArriveEarly'][i] = variables['ArriveEarly'][i].varValue
                self.sol['ArriveLate'][i] = variables['ArriveLate'][i].varValue
                for j in self.dim:
                    self.sol['x'][i][j] = variables['x'][i][j].varValue
            self.save_to_file(self.sol,self.output_dir)
            return None
        else:
            return None

    def add_obj(self, mdl, variables):
        if not variables:
            logging.error('Variables Empty')
            raise Exception('Illegal Argument(s) Exception')
        mdl.objective += self.wts['dist']*pl.lpSum((self.data.dist[i,j]*variables.get('x')[i][j]
                                  for i,j in itertools.product(self.dim,self.dim))) # add distince obj
        mdl.objective += self.wts['time']*pl.lpSum((self.data.time[i,j]*variables.get('x')[i][j]
                                  for i,j in itertools.product(self.dim,self.dim)))  # add time obj
        mdl.objective += 10*self.wts['time'] * pl.lpSum((variables.get('ArriveEarly')[i]
                                                         for i in self.dim))  # add arrive early penalty
        mdl.objective += 1000*self.wts['time']*pl.lpSum((variables.get('ArriveLate')[i]
                                                           for i in self.dim))  # add arrive late penalty
        print('Objective added Successfully')
        return mdl

    def add_routing_constraints(self,mdl,var):
        for j in self.dim:
            mdl += pl.lpSum([var['x'][i][j] for i in self.dim]) == 1,"flow_balance_constraints_{}".format(j)
        for i in self.dim:
            mdl += pl.lpSum([var['x'][i][j] for j in self.dim]) == 1, 'flow_balance_constraints2_{}'.format(i)
        for i in self.dim:
            mdl += var['x'][i][i] == 0,'No_self_tour_{}'.format(i)
        for i in self.dim:
            if i == self.SourceIndex:
                mdl += var['u'][i] == 1, 'Subtour_elimination_{}'.format(i)
            else:
                mdl += var['u'][i] >= 2, 'Subtour_elimination_{}'.format(i)
        for i in self.dim:
            for j in self.dim:
                if (i != self.SourceIndex) & (j != self.SourceIndex) & (i!=j):
                    mdl += var['u'][i] - var['u'][j]<= self.Nloc - self.Nloc*var['x'][i][j] -1, 'Subtour_elimination2_{}_{}'.format(i,j)
        logger.info('Add Routing Constraints')
        return mdl

    def add_time_constraints(self,mdl,var):
        for i in self.dim:
            if i == self.SourceIndex:
                mdl += var['t'][i] <= self.data.timelim[i][1]
                mdl += var['t'][i] >= self.data.timelim[i][0]
            else:
                #time at other node should later then start time + direct driveing time
                mdl += var['t'][i] >= var['t'][self.SourceIndex] + self.data.time[self.SourceIndex][i]
        for i in self.dim:
            for j in self.dim:
                if (i != self.SourceIndex) & (j != self.SourceIndex) & (i != j):
                    mdl += var['t'][i] + self.data.time[i][j] <= var['t'][j] + self.bigM*(1-var['x'][i][j])

        for i in self.dim:
            if self.time_constraints:
                mdl += var['t'][i] + var['ArriveEarly'][i] >= self.data.timelim[i][0], 'Arrive Early{}'.format(i)
                mdl += var['t'][i] <= self.data.timelim[i][1] + var['ArriveLate'][i], 'Arrive Late{}'.format(i)
            else:
                continue
        return mdl

    def save_to_file(self,sol,output_dir):
        output = []
        drive_time = np.empty(self.Nloc)
        drive_dist = np.empty(self.Nloc)
        for i in self.dim:
            drive_time[i] = np.around(np.dot(self.data.time[i],sol['x'][i]),2)
            drive_dist[i] = np.around(np.dot(self.data.dist[i],sol['x'][i])/1609.34,2)

        for i in self.dim:
            output.append([i+1,np.floor(sol['u'][i]-1+0.5),
                           round(drive_time[i]/60,2),drive_dist[i],sol['t'][i],sol['ArriveEarly'][i],
                           sol['ArriveLate'][i]])
        output_df = pd.DataFrame(output,columns=['Node','Sequence',
                                                 'Driving_Time','Distance','Arrival_time',
                                                 'Arrive_Early','Arrive_Late'])
        output_df.sort_values(by=['Sequence'],axis=0,inplace=True)
        output_df.to_csv(output_dir,index=False)
        print('Result saved at {}'.format(output_dir))

if __name__ == '__main__':
    wts = {'dist':0,'time':1}
    Data = Get_Inputs('data')
    optimizer = Traditional_Optimizer(Data,wts,VisitTime=0,output_dir='Results',time_constraint=True)
    optimizer.get_shortest_route()