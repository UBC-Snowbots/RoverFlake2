"""
Created by: Cameron Basara
Date Created: 2/11/2024
Purpose: To retrieve data from a usb-drive, and intepret the data regarding a CIRC 2024 Search and Rescue Task
Log file format: [ISO 8601 Timestamp] [Severity] Cell [Number] Diagnostic [OK/FAIL] [message]
                  2023-12-12T23:45:06.3743Z WARN Cell 2 Diagnostic FAIL Short Detected.
"""

import os
import re

def main(args=None):
    # Retrieve data from file
    script_path = os.path.dirname(__file__)  # Gets the directory where the script is located
    file_path = os.path.join(script_path, '../logs/lander.log')  # Navigate to the target file
    data =  r'%s' % file_path

    with open(data) as f:
        # Declare diagnostic var
        message = []

        # Iterate lines in f
        for line in f:
            hold = re.search("FAIL", line) 
            if hold is not None:
                message.append(line[:-1])
        
        print(message)

if __name__ == '__main__':
    main()








