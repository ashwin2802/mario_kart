#!/usr/bin/python3
import os

model_token = "MODEL_NAME"
sdf_token = "model.sdf"
config_token = "model.config"

template_sdf = open(sdf_token, 'r').read()
template_config = open(config_token, 'r').read()

os.chdir("../models")
models = os.listdir()

for model in models:
    os.chdir(model)
    with open(sdf_token, 'w') as sdf_file:
        sdf_file.write(template_sdf.replace(model_token, model))
    with open(config_token, 'w') as config_file:
        config_file.write(template_config.replace(model_token, model))
    os.chdir("..")
