import abc

class ModelControllerInterface(abc.ABC):

    @abc.abstractclassmethod
    def load_model(chp_path: str):
        pass

    @abc.abstractclassmethod
    def get_input():
        pass

    @abc.abstractclassmethod
    def pre_process_input():
        pass

    @abc.abstractclassmethod
    def get_action():
        pass