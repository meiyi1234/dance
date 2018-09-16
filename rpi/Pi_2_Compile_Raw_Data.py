# -*- coding: utf-8 -*-
"""
Created on Wed Nov  1 16:39:56 2017

@author: diary
"""
import pandas as pd
import os

# Config.ini
reshapeBy = 50 # Set number of inputs per sample for Machine Learning
#sys.stdout = open("OutputComb.txt", "w") # Set print command to print to file

# Variable Declarations
firstFile = True

# Declare column headers
cols = [list(range(1, (9*reshapeBy)+2))] # 1-451, 450 data + dance move
#fullDF = pd.DataFrame(columns=cols)


for filename in os.listdir('RawData'):
    if (firstFile): # First declaration of fullDF for the first file read
        firstFile = False
        pdtestdata = pd.read_csv("RawData\\"+str(filename), delimiter=',')
        pdtestdata = pdtestdata.drop(pdtestdata.columns[0], axis=1) # Remove ID
        fullDF = pdtestdata.reset_index(drop=True)
        del pdtestdata
    else: # Append data from remaining files to fullDF
        pdtestdata = pd.read_csv("RawData\\"+str(filename), delimiter=',')
        pdtestdata = pdtestdata.drop(pdtestdata.columns[0], axis=1) # Remove ID
        fullDF = pd.concat([fullDF, pdtestdata], axis=0)
        del pdtestdata

print(fullDF)

# Save compiled raw data file
fullDF.to_csv('processed_data.csv', sep=',')
