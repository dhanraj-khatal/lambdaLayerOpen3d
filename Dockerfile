FROM 049363449871.dkr.ecr.ap-south-1.amazonaws.com/dev-open-3d-base-image:latest
WORKDIR /root
RUN chmod o+rx /root
COPY ./constants.py ./constants.py
COPY ./AiLogic ./AiLogic/
COPY ./utils ./utils/
COPY ./requirements.txt ./requirements.txt
COPY ./.env ./.env
COPY ./samples ./samples
COPY ./stl ./stl
COPY ./emboss_numbers ./emboss_numbers

ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update
RUN apt-get install -y libglib2.0-0

RUN conda install -y scipy

RUN pip install --default-timeout=300 -r requirements.txt -t /opt/python/

CMD cd /opt && zip -r9 /app/requests-layer.zip .