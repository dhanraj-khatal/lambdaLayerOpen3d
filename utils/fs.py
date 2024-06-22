import logging
import os
import gc
import sys
import shutil
import requests
from botocore.exceptions import ClientError
import constants
from datetime import datetime
import json

# S3 - Upload file


def upload_file(filename, presignedUrl):
    try:
        logging.info("uploading from :" + filename)
        logging.info("uploading to :" + presignedUrl)
        with open(filename, 'rb') as f:
            response = requests.put(presignedUrl, f)
    except ClientError as e:
        logging.error("error: " + e)
        return None
    return response

# OS - Delete file


def delete_file(filename):
    try:
        logging.info('deleting file' + filename)
        if(os.path.exists(filename)):
            os.remove(filename)
            gc.collect()
    except Exception as e:
        print(e)
       

# OS - List dir


def listDir(path):
    dirs = os.listdir(path)
    fileArray = []
    for file in dirs:
        print(file)
        fileArray.append(file)
        logging.info('**' + file)
    return fileArray
# Delete folder recursive with contents


def delete_folder(foldername):
    logging.info('deleting folder' + foldername)
    if(os.path.exists(foldername) and foldername.lower() != constants.LOCAL_FOLDER.lower()):
        shutil.rmtree(foldername)
        gc.collect()

# Create Temp folder and return name of newly created dir


def create_temp_folder(folderExtension = None):
    logging.info('creating temp folder')
    if folderExtension is None:
        dir_name = os.path.join(constants.LOCAL_FOLDER, datetime.utcnow().strftime('%Y_%m_%d_%H_%M_%S_%f'))
    else:
        dir_name = os.path.join(constants.LOCAL_FOLDER, datetime.utcnow().strftime('%Y_%m_%d_%H_%M_%S_%f'), folderExtension)
    try:
        if not os.path.isdir(dir_name):
            os.makedirs(dir_name)
            logging.info(dir_name + ' created!')
        else:
            logging.info(dir_name + ' exists!')
        return dir_name
    except OSError as e:
        logging.error("error in creating dir", e)
        return None

def save_as_json(data, file_name):
    out_file = open(file_name, "w")
    json.dump(data, out_file, indent=4)
    out_file.close()
