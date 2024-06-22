import json
import boto3
import logging
from botocore.exceptions import ClientError
# create lambda client
client = boto3.client('lambda',region_name= 'ap-south-1')

def invoke_lambda(lambdaName,payload):
    try:
        result = client.invoke(FunctionName = lambdaName,
        InvocationType='RequestResponse',
        Payload=payload)
        print(result)
        # range = result['Payload'].read()
        # api_response = json.loads(range)
        return result
    except ClientError as e:
        logging.error(e)
        return None