from data_values import export_data
from settings import EXPORT_FILE_ADDRESS

def exportString(fileName:str, string:str):
    '''Append a specified string to the specified file.'''
    with open(fileName + "csv", "a") as outfile:
        outfile.write(string)

def exportValue(value:str):
    '''Export a specified string value to the export file.'''
    exportString(EXPORT_FILE_ADDRESS, value + "\n")

def exportHeader():
    '''Write the header string for the current export data to the export file.'''
    exportValue(export_data.getHeaderString())

def exportData():
    '''Write the current export data string to the export file.'''
    exportValue(export_data.getExportString())

def clearExportFile():
    '''Clear the export file from all previous data.'''
    with open(EXPORT_FILE_ADDRESS + ".csv", "w") as outfile:
        outfile.write("")