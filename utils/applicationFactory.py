from flask import Flask
from flask_cors import CORS
import logging

logging.getLogger('flask_cors').level = logging.DEBUG


class Factory:
    def __init__(self):
        self.app = None

    def create_app(self):
        self.app = Flask(__name__)

def get_app():
    factory = Factory()
    if factory.app == None:
        factory.create_app()
        cors = CORS(factory.app, resources={r"/v1/*": {"origins": "*"}})
    return factory.app
