import logging
import boto3
from botocore.exceptions import ClientError
import os
import json
import constants
from flask import jsonify
import threading

s3_client = boto3.client('s3', region_name='ap-south-1')

def upload_file(file_name, bucket, object_name=None):
    """Upload a file to an S3 bucket

    :param file_name: File to upload
    :param bucket: Bucket to upload to
    :param object_name: S3 object name. If not specified then file_name is used
    :return: True if file was uploaded, else False
    """

    # If S3 object_name was not specified, use file_name
    if object_name is None:
        object_name = os.path.basename(file_name)

    # Upload the file
    try:
        response = s3_client.upload_file(file_name, bucket, object_name)
        print("file Uploaded")
    except ClientError as e:
        logging.error(e)
        return False
    return True

def upload_folder(s3bucket, inputDir, s3Path):
    print("Uploading results to s3 initiated...")
    print("Local Source:", inputDir)
    try:
       # This is my path
        path = inputDir
        for (root, dirs, file) in os.walk(path):
            for f in file:
                file_to_upload = os.path.join(inputDir, f)
                print(s3Path+f)
                s3_client.upload_file(file_to_upload, s3bucket, s3Path+f)
                print("folder uploaded")
    except Exception as e:
        print(" ... Failed!! Quitting Upload!!")
        print(e)
        raise e

def upload_step_folder(s3bucket, inputDir, s3Path):
    print("Uploading results to s3 initiated...")
    print("Local Source:", inputDir)
    try:
       # This is my path
        path = inputDir
        files = list()
        fileUploadThreads = list()
        # dirs=directories
        for (root, dirs, file) in os.walk(path):
            for f in file:
                file_to_upload = os.path.join(inputDir, f)
                files.append({ 'file_to_upload': file_to_upload, 's3bucket': s3bucket, 's3Path': s3Path+f })
                if len(files) == 100 :
                    fileThread = threading.Thread(target=upload_folder_files_thread, args=(files,))
                    files = list()
                    fileUploadThreads.append(fileThread)
                    fileThread.start()
        if len(files) != 0 :
            fileThread = threading.Thread(target=upload_folder_files_thread, args=(files,))
            fileUploadThreads.append(fileThread)
            fileThread.start()
        for thread in fileUploadThreads:
            thread.join()
    except Exception as e:
        print(" ... Failed!! Quitting Upload!!")
        print(e)
        raise e

def upload_folder_files_thread(files):
    for file in files:
        print(file['s3Path'])
        s3_client.upload_file(file['file_to_upload'], file['s3bucket'], file['s3Path'])
        print("folder uploaded")

def download_file(BUCKET_NAME, OBJECT_NAME, FILE_NAME):
    try:
        print('Downloading from ' + OBJECT_NAME)
        logging.info('Downloading from ' + OBJECT_NAME)
        downlodedFile = s3_client.download_file(BUCKET_NAME, OBJECT_NAME, FILE_NAME)
        logging.info('Downloaded to '+FILE_NAME)
        return True
    except ClientError as e:
        logging.error(e)
        return e


def download_s3_folder(bucket_name, s3_folder, local_dir):
    print('inside download_s3_folder ')
    """
    Download the contents of a folder directory
    Args:
        bucket_name: the name of the s3 bucket
        s3_folder: the folder path in the s3 bucket
        local_dir: a relative or absolute directory path in the local file system
    """
    try:
        s3Path = bucket_name+'/'+s3_folder
        result = s3_client.list_objects_v2(
            Bucket=bucket_name, Prefix=s3_folder, Delimiter='/')
        logging.info(f'files to download: {result}')
        for o in result['Contents']:
            try:
                download_file(bucket_name, o['Key'],
                          local_dir+'/'+os.path.basename(o['Key']))
            except Exception as e:
                logging.error(e)

    except Exception as e:
        logging.error(e)
        return jsonify({'result': 'Error'})


def delete_file(bucket_name, key):
    try:
        logging.info('Deleting ' + key + ' from ' + bucket_name)
        response = s3_client.delete_object(
            Bucket=bucket_name,
            Key=key)
        logging.info(response)
        return True
    except ClientError as e:
        logging.error(e)
        return False


def get_s3_keys(bucket, prefix):
    try:
        """Get a list of keys in an S3 bucket."""
        files = []
        print(bucket, prefix)
        resp = s3_client.list_objects_v2(Bucket=bucket, Prefix=prefix)
        print(resp)
        if 'Contents' in resp.keys():
            for obj in resp['Contents']:
                files.append(obj['Key'])

        else:
              return files 
        return files
    except ClientError as e:
        logging.error(e)
        return files #return empty array in case of content not found


def delete_folder(bucket, prefix):
    try:
        filesArray = get_s3_keys(bucket, prefix)
        print(filesArray)
        for file in filesArray:
            print(file)
            delete_file(bucket, file)
            print('file deleted')
            logging.info('file deleted')
        return True
    except ClientError as e:
        logging.error(e)
        return False


def create_presigned_url(bucket_name, object_name, expiration=600):
    # Choose AWS CLI profile, If not mentioned, it would take default
    # boto3.setup_default_session(profile_name='personal')
    # Generate a presigned URL for the S3 object
    try:
        response = s3_client.generate_presigned_url('get_object',
                                                    Params={'Bucket': bucket_name,
                                                            'Key': object_name},
                                                    ExpiresIn=expiration)
    except Exception as e:
        print(e)
        logging.error(e)
        return "Error"
    # The response contains the presigned URL
    print(response)
    return response

