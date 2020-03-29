import gspread
from oauth2client.service_account import ServiceAccountCredentials
import googlemaps
from datetime import datetime
import pickle
import os.path
from googleapiclient.discovery import build
from google_auth_oauthlib.flow import InstalledAppFlow
from google.auth.transport.requests import Request
import pandas as pd
import numpy as np
class read_write_gsheet:
    def __init__(self):
        SCOPES = ['https://www.googleapis.com/auth/spreadsheets.readonly']
        creds = None
        # The file token.pickle stores the user's access and refresh tokens, and is
        # created automatically when the authorization flow completes for the first
        # time.
        if os.path.exists('token.pickle'):
            with open('token.pickle', 'rb') as token:
                creds = pickle.load(token)
        # If there are no (valid) credentials available, let the user log in.
        if not creds or not creds.valid:
            if creds and creds.expired and creds.refresh_token:
                creds.refresh(Request())
            else:
                flow = InstalledAppFlow.from_client_secrets_file(
                    'open-house-routing-7cff1b86d3e9.json', SCOPES)
                creds = flow.run_local_server(port=0)
            # Save the credentials for the next run
            with open('token.pickle', 'wb') as token:
                pickle.dump(creds, token)
        self.service = build('sheets', 'v4', credentials=creds)

    def read_sheet(self,spreadsheet_id,range_name="Sheet1"):
        self.result = self.service.spreadsheets().values().get(
            spreadsheetId=spreadsheet_id, range=range_name).execute()
        values = self.result.get('values', [])
        return values

    def append_values(self, values,spreadsheet_id,range_name="Sheet1"):
        body = {
            'values': values
        }
        self.result = self.service.spreadsheets().values().append(
            spreadsheetId=spreadsheet_id, range=range_name, body=body).execute()

class get_time_dist_matrix:
    def __init__(self,home_addree = None,api_key='file'):
        if api_key=='file':
            with open('credential') as file:
                api_key = file.readline()
            file.close()
        self.gmaps = googlemaps.Client(key=api_key)
        if not home_addree:
            self.home_address = '2007 Meadowbrook Way Dr, Chesterfield, MO 63017'
        else:
            self.home_address = home_addree
    def dist_matrix_json(self,origins,destinations,**kwargs):
        client = self.gmaps
        from googlemaps import convert
        paras = {"origins": convert.location_list(origins),
                "destinations": convert.location_list(destinations),
                'mode':"driving",
                 'units':"imperial"
                 }
        for i in paras.keys():
            if kwargs.get(i):
                paras[i] = kwargs.get(i)
        return client._request("/maps/api/distancematrix/json", paras)

    def address_to_dist_matrix(self,addreses):
        addreses = [self.home_address] + addreses.tolist()
        dim = len(addreses)
        dist_mat = np.zeros((dim,dim),dtype=int)
        time_mat = np.zeros((dim,dim),dtype=int)
        response = self.dist_matrix_json(addreses,addreses)
        for i,row in enumerate(response['rows']):
            elements = row.get('elements')
            for j,element in enumerate(elements):
                assert element.get('status')=='OK',"API status is not correct"
                dist_mat[i,j] = element.get('distance').get('value')
                time_mat[i,j] = element.get('duration').get('value')
        return dist_mat,time_mat

    def get_time_lim(self,timelim_df):
        def process_time(val):
            lb, ub = str(val).split('-')
            lb = datetime.strptime(lb, '%I%p')
            ub = datetime.strptime(ub, '%I%p')
            zero_time = datetime.strptime('00', '%H')
            lb = int((lb-zero_time).total_seconds())
            ub = int((ub-zero_time).total_seconds())
            return [lb,ub]
        df = timelim_df.apply(lambda x: process_time(x)).tolist()
        df = [[0,999999]] + df
        return df

    def read_save(self,dir):
        houselist = pd.read_csv(os.path.join(dir,'house_list.csv'))
        self.dist_matrix,self.time_matrix = self.address_to_dist_matrix(houselist['Address'])
        self.timelim = self.get_time_lim(houselist['Time'])
        self.timelim = np.asarray(self.timelim,dtype=np.int)
        np.savetxt(os.path.join(dir,'dist_matrix.csv'),self.dist_matrix.astype(int),delimiter=',',fmt='%i')
        np.savetxt(os.path.join(dir, 'time_matrix.csv'), self.time_matrix.astype(int), delimiter=',',fmt='%i')
        np.savetxt(os.path.join(dir, 'time_constraint.csv'), self.timelim.astype(int), delimiter=',',fmt='%i')
        print('Data Saved!')

if __name__=="__main__":
    #gsheet =  read_write_gsheet()
    #openhouselist = gsheet.read_sheet(spreadsheet_id="5/21 Open House List",range_name="Sheet1")
    gapi = get_time_dist_matrix(api_key='file')
    gapi.read_save(dir='data')
