import serial
import openpyxl
import re
from datetime import datetime

# Open a xlsx file for writing
timestamp = datetime.now().strftime("%m%d_%H%M")
xlsx_filename = f"Time_diff_{timestamp}.xlsx"
wb = Workbook()
ws = wb.active

data_list=[]

csv_writer = csv.writer(csvfile)

ser = serial.Serial('COM9', 115200, timeout=1)  

try:
    for i in range(100):
        arduino_data = ser.readline().decode('utf-8')
        match1 = int(re.search(r'\d+', arduino_data).group())
        data_list.append(match1)            
        print(match1)
           

except KeyboardInterrupt:
    # Close the serial port when the user interrupts the program
    ser.close()
    print("Serial port closed.")

# Calculate mean
mean_value = sum(data_list) / len(data_list)

# Calculate standard deviation
sum_squared_diff = sum((x - mean_value) ** 2 for x in data_list)
std_deviation = (sum_squared_diff / len(data_list)) ** 0.5
    
# Write necessary data to xlsx
ws.append(['Time Difference per 100 change in Frac'])
for integer in data_list:
    ws.append([integer])


