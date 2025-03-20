import abc


class ModelControllerInterface(abc.ABC):

    @abc.abstractclassmethod
    def load_model(self, chp_path: str):
        pass

    @abc.abstractclassmethod
    def get_input(self):
        pass

    @abc.abstractclassmethod
    def pre_process_input(self):
        pass

    @abc.abstractclassmethod
    def get_action(self):
        pass
