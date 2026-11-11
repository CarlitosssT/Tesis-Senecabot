from .base import ControlFunction
from .default import DefaultControl
from .pd import PDControl
from .tf_pd import TFPDControl   # [SENECA LOCAL CHANGE] PD through the identified MyActuator transfer function

# register all control functions
DefaultControl.register()
PDControl.register()
TFPDControl.register()
