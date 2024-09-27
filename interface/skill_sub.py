import zmq
import time
import yaml
import numpy as np
from protobuf import skill_pb2

class SkillSubscriber:
    def __init__(self, address: str) -> None:
        self.context = zmq.Context()
        self.subscriber = self.context.socket(zmq.SUB)
        self.subscriber.connect(address)
        self.subscriber.setsockopt_string(zmq.SUBSCRIBE, "")
        self.skill_list = []

    def receive_message(self):
        skill = skill_pb2.Skill()
        skill.ParseFromString(self.subscriber.recv())

        self.skill_list[:] = skill.skill

def main():
    with open("../config/address.yaml", "r") as f:
        address = yaml.load(f.read(), Loader=yaml.Loader)["robot_skill"]

    skill_subscriber = SkillSubscriber(address=address)

    while True:
        skill_subscriber.receive_message()
        print(skill_subscriber.skill_list)

if __name__ == "__main__":
    main()