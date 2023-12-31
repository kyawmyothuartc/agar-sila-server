# Generated by sila2.code_generator; sila2.__version__: 0.10.4
from __future__ import annotations

from typing import TYPE_CHECKING

from sila2.server import MetadataDict
from utils.Application import SiLA as AgarSila
import logging

from ..generated.agarcontroller import (
    AgarControllerBase,
    CallSubroutine_Responses,
    IdentifyColony_Responses,
    ResetTipCount_Responses,
    RingLightLamp_Responses,
    RingLightMotor_Responses,
    RobotControlStartProcess_Responses,
)

if TYPE_CHECKING:
    from ..server import Server


class AgarControllerImpl(AgarControllerBase):
    def __init__(self, parent_server: Server) -> None:
        super().__init__(parent_server=parent_server)
        self.agr = AgarSila()
        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(logging.INFO)

    def get_GetTipCount(self, *, metadata: MetadataDict) -> int:
        response = self.agr.getTipCount()
        self.logger.info(f"get_GetTipCount - {response}")
        return response
        #raise NotImplementedError  # TODO

    def get_CaptureColony(self, *, metadata: MetadataDict) -> str:
        response = self.agr.colony_capture()
        self.logger.info(f"get_CaptureColony - {response}")
        return response
        #raise NotImplementedError  # TODO

    def RingLightLamp(self, OnOff: bool, *, metadata: MetadataDict) -> RingLightLamp_Responses:
        response = self.agr.ringLightLamp(OnOff)
        self.logger.info(f"RingLightLamp - {response}")
        return response
        #raise NotImplementedError  # TODO

    def RingLightMotor(self, OnOff: bool, *, metadata: MetadataDict) -> RingLightMotor_Responses:
        response = self.agr.ringLightMotor(OnOff)
        self.logger.info(f"RingLightMotor - {response}")
        return response
        #raise NotImplementedError  # TODO

    def ResetTipCount(self, TipCount: int, *, metadata: MetadataDict) -> ResetTipCount_Responses:
        TipCount = 96
        self.logger.info(f"SetTipCount Call TipCount - {TipCount}")
        response = self.agr.setTipCount(TipCount)
        self.logger.info(f"SetTipCount Response - {response}")
        return response
        #raise NotImplementedError  # TODO

    def CallSubroutine(self, Routine: str, Arg1: str, Arg2: str, *, metadata: MetadataDict) -> CallSubroutine_Responses:
        self.logger.info(f"CallSubroutine Call Routine - {Routine}, Arg_1 - {Arg1}, Arg_2 - {Arg2}")
        response = self.agr.callSubroutine( Routine, Arg1, Arg2)
        self.logger.info(f"CallSubroutine Response - {response}")
        return response
        #raise NotImplementedError  # TODO

    def RobotControlStartProcess(
        self, OutputType: str, StreakingType: str, TransferType: str, Coordinates: str, *, metadata: MetadataDict
    ) -> RobotControlStartProcess_Responses:
        self.logger.info(f"RobotControlStartProcess Call OutputType - {OutputType}, StreakingType - {StreakingType}, TransferType - {TransferType}, Coordinates - {Coordinates}")
        response = self.agr.robot_control_START_process(OutputType, StreakingType, TransferType, Coordinates)
        self.logger.info(f"RobotControlStartProcess Response - {response}")
        return response
        #raise NotImplementedError  # TODO

    def IdentifyColony(self, ColonyValue: int, *, metadata: MetadataDict) -> IdentifyColony_Responses:
        self.logger.info(f"IdentifyColony Call Routine - {ColonyValue}")
        image, coordinate = self.agr.colony_identify(ColonyValue)
        self.logger.info(f"IdentifyColony image - {image} , Coordinates: {coordinate}")
        return image, coordinate
        #raise NotImplementedError  # TODO
