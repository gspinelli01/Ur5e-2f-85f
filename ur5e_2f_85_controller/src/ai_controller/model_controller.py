import abc

class ModelControllerInterface(abc.ABC):

    @abc.abstractclassmethod
    def load_model(chp_path):
        pass


    @abc.abstractclassmethod
    def get_action():
        pass