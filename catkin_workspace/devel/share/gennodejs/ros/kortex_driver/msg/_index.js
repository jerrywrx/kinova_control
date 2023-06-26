
"use strict";

let KortexError = require('./KortexError.js');
let ApiOptions = require('./ApiOptions.js');
let SubErrorCodes = require('./SubErrorCodes.js');
let ErrorCodes = require('./ErrorCodes.js');
let PositionCommand = require('./PositionCommand.js');
let SafetyIdentifierBankA = require('./SafetyIdentifierBankA.js');
let CoggingFeedforwardModeInformation = require('./CoggingFeedforwardModeInformation.js');
let ActuatorConfig_ControlMode = require('./ActuatorConfig_ControlMode.js');
let ControlLoop = require('./ControlLoop.js');
let CommandModeInformation = require('./CommandModeInformation.js');
let TorqueOffset = require('./TorqueOffset.js');
let CoggingFeedforwardMode = require('./CoggingFeedforwardMode.js');
let FrequencyResponse = require('./FrequencyResponse.js');
let ActuatorConfig_ControlModeInformation = require('./ActuatorConfig_ControlModeInformation.js');
let EncoderDerivativeParameters = require('./EncoderDerivativeParameters.js');
let ActuatorConfig_SafetyLimitType = require('./ActuatorConfig_SafetyLimitType.js');
let ControlLoopParameters = require('./ControlLoopParameters.js');
let CustomDataIndex = require('./CustomDataIndex.js');
let VectorDriveParameters = require('./VectorDriveParameters.js');
let CommandMode = require('./CommandMode.js');
let StepResponse = require('./StepResponse.js');
let AxisPosition = require('./AxisPosition.js');
let CustomDataSelection = require('./CustomDataSelection.js');
let ControlLoopSelection = require('./ControlLoopSelection.js');
let Servoing = require('./Servoing.js');
let LoopSelection = require('./LoopSelection.js');
let AxisOffsets = require('./AxisOffsets.js');
let TorqueCalibration = require('./TorqueCalibration.js');
let ActuatorConfig_ServiceVersion = require('./ActuatorConfig_ServiceVersion.js');
let RampResponse = require('./RampResponse.js');
let ActuatorCyclic_ServiceVersion = require('./ActuatorCyclic_ServiceVersion.js');
let ActuatorCyclic_Feedback = require('./ActuatorCyclic_Feedback.js');
let CommandFlags = require('./CommandFlags.js');
let ActuatorCyclic_Command = require('./ActuatorCyclic_Command.js');
let ActuatorCyclic_MessageId = require('./ActuatorCyclic_MessageId.js');
let ActuatorCyclic_CustomData = require('./ActuatorCyclic_CustomData.js');
let StatusFlags = require('./StatusFlags.js');
let ControllerHandle = require('./ControllerHandle.js');
let Sequence = require('./Sequence.js');
let ArmStateNotification = require('./ArmStateNotification.js');
let NetworkType = require('./NetworkType.js');
let GpioPinConfiguration = require('./GpioPinConfiguration.js');
let WifiConfigurationList = require('./WifiConfigurationList.js');
let ServoingModeInformation = require('./ServoingModeInformation.js');
let CartesianLimitation = require('./CartesianLimitation.js');
let FirmwareBundleVersions = require('./FirmwareBundleVersions.js');
let ConfigurationNotificationEvent = require('./ConfigurationNotificationEvent.js');
let MapGroup = require('./MapGroup.js');
let SoundType = require('./SoundType.js');
let TrajectoryErrorIdentifier = require('./TrajectoryErrorIdentifier.js');
let TrajectoryInfo = require('./TrajectoryInfo.js');
let JointTrajectoryConstraintType = require('./JointTrajectoryConstraintType.js');
let TransformationMatrix = require('./TransformationMatrix.js');
let ControllerElementEventType = require('./ControllerElementEventType.js');
let WrenchCommand = require('./WrenchCommand.js');
let ControlModeNotificationList = require('./ControlModeNotificationList.js');
let WifiConfiguration = require('./WifiConfiguration.js');
let OperatingMode = require('./OperatingMode.js');
let Base_ControlModeInformation = require('./Base_ControlModeInformation.js');
let FirmwareComponentVersion = require('./FirmwareComponentVersion.js');
let PreComputedJointTrajectoryElement = require('./PreComputedJointTrajectoryElement.js');
let JointTorque = require('./JointTorque.js');
let WifiInformation = require('./WifiInformation.js');
let JointNavigationDirection = require('./JointNavigationDirection.js');
let ActionExecutionState = require('./ActionExecutionState.js');
let MapElement = require('./MapElement.js');
let ControllerEventType = require('./ControllerEventType.js');
let JointsLimitationsList = require('./JointsLimitationsList.js');
let UserNotification = require('./UserNotification.js');
let ProtectionZoneNotification = require('./ProtectionZoneNotification.js');
let MapGroupList = require('./MapGroupList.js');
let SequenceInformation = require('./SequenceInformation.js');
let SequenceTasksConfiguration = require('./SequenceTasksConfiguration.js');
let ActionList = require('./ActionList.js');
let ServoingMode = require('./ServoingMode.js');
let TransformationRow = require('./TransformationRow.js');
let JointLimitation = require('./JointLimitation.js');
let GpioConfigurationList = require('./GpioConfigurationList.js');
let SequenceTaskConfiguration = require('./SequenceTaskConfiguration.js');
let NavigationDirection = require('./NavigationDirection.js');
let ActuatorInformation = require('./ActuatorInformation.js');
let ServoingModeNotification = require('./ServoingModeNotification.js');
let ConfigurationChangeNotificationList = require('./ConfigurationChangeNotificationList.js');
let ZoneShape = require('./ZoneShape.js');
let SequenceTasks = require('./SequenceTasks.js');
let MappingInfoNotification = require('./MappingInfoNotification.js');
let ProtectionZoneHandle = require('./ProtectionZoneHandle.js');
let OperatingModeNotificationList = require('./OperatingModeNotificationList.js');
let BridgePortConfig = require('./BridgePortConfig.js');
let SnapshotType = require('./SnapshotType.js');
let SequenceInfoNotificationList = require('./SequenceInfoNotificationList.js');
let ControllerInputType = require('./ControllerInputType.js');
let TrajectoryErrorElement = require('./TrajectoryErrorElement.js');
let Orientation = require('./Orientation.js');
let FactoryNotification = require('./FactoryNotification.js');
let RequestedActionType = require('./RequestedActionType.js');
let ActionEvent = require('./ActionEvent.js');
let GripperCommand = require('./GripperCommand.js');
let EventIdSequenceInfoNotification = require('./EventIdSequenceInfoNotification.js');
let SwitchControlMapping = require('./SwitchControlMapping.js');
let Base_Position = require('./Base_Position.js');
let TwistLimitation = require('./TwistLimitation.js');
let RobotEventNotification = require('./RobotEventNotification.js');
let Base_GpioConfiguration = require('./Base_GpioConfiguration.js');
let ActionNotificationList = require('./ActionNotificationList.js');
let UserList = require('./UserList.js');
let OperatingModeNotification = require('./OperatingModeNotification.js');
let WifiEncryptionType = require('./WifiEncryptionType.js');
let MappingList = require('./MappingList.js');
let ControllerNotification = require('./ControllerNotification.js');
let Wrench = require('./Wrench.js');
let AngularWaypoint = require('./AngularWaypoint.js');
let KinematicTrajectoryConstraints = require('./KinematicTrajectoryConstraints.js');
let ConfigurationChangeNotification_configuration_change = require('./ConfigurationChangeNotification_configuration_change.js');
let TrajectoryErrorType = require('./TrajectoryErrorType.js');
let Base_RotationMatrix = require('./Base_RotationMatrix.js');
let Base_Stop = require('./Base_Stop.js');
let WaypointValidationReport = require('./WaypointValidationReport.js');
let WrenchMode = require('./WrenchMode.js');
let Xbox360DigitalInputIdentifier = require('./Xbox360DigitalInputIdentifier.js');
let FullIPv4Configuration = require('./FullIPv4Configuration.js');
let Faults = require('./Faults.js');
let PasswordChange = require('./PasswordChange.js');
let Base_ControlModeNotification = require('./Base_ControlModeNotification.js');
let NetworkHandle = require('./NetworkHandle.js');
let Waypoint = require('./Waypoint.js');
let TrajectoryContinuityMode = require('./TrajectoryContinuityMode.js');
let WristDigitalInputIdentifier = require('./WristDigitalInputIdentifier.js');
let AdvancedSequenceHandle = require('./AdvancedSequenceHandle.js');
let BridgeIdentifier = require('./BridgeIdentifier.js');
let GripperRequest = require('./GripperRequest.js');
let SequenceTask = require('./SequenceTask.js');
let ShapeType = require('./ShapeType.js');
let ControllerType = require('./ControllerType.js');
let Xbox360AnalogInputIdentifier = require('./Xbox360AnalogInputIdentifier.js');
let CartesianTrajectoryConstraint_type = require('./CartesianTrajectoryConstraint_type.js');
let IKData = require('./IKData.js');
let Snapshot = require('./Snapshot.js');
let Base_JointSpeeds = require('./Base_JointSpeeds.js');
let MapEvent = require('./MapEvent.js');
let ProtectionZone = require('./ProtectionZone.js');
let ControllerElementHandle_identifier = require('./ControllerElementHandle_identifier.js');
let JointTorques = require('./JointTorques.js');
let ConstrainedOrientation = require('./ConstrainedOrientation.js');
let CartesianTrajectoryConstraint = require('./CartesianTrajectoryConstraint.js');
let ProtectionZoneNotificationList = require('./ProtectionZoneNotificationList.js');
let ProtectionZoneEvent = require('./ProtectionZoneEvent.js');
let Twist = require('./Twist.js');
let ControllerEvent = require('./ControllerEvent.js');
let ControllerConfigurationMode = require('./ControllerConfigurationMode.js');
let GripperMode = require('./GripperMode.js');
let AdmittanceMode = require('./AdmittanceMode.js');
let Point = require('./Point.js');
let SequenceTasksRange = require('./SequenceTasksRange.js');
let Base_CapSenseConfig = require('./Base_CapSenseConfig.js');
let BridgeList = require('./BridgeList.js');
let ActionHandle = require('./ActionHandle.js');
let BridgeStatus = require('./BridgeStatus.js');
let ConstrainedJointAngles = require('./ConstrainedJointAngles.js');
let Mapping = require('./Mapping.js');
let Finger = require('./Finger.js');
let GpioBehavior = require('./GpioBehavior.js');
let ActivateMapHandle = require('./ActivateMapHandle.js');
let RobotEvent = require('./RobotEvent.js');
let Delay = require('./Delay.js');
let SequenceList = require('./SequenceList.js');
let Waypoint_type_of_waypoint = require('./Waypoint_type_of_waypoint.js');
let Admittance = require('./Admittance.js');
let JointTrajectoryConstraint = require('./JointTrajectoryConstraint.js');
let ServoingModeNotificationList = require('./ServoingModeNotificationList.js');
let JointSpeed = require('./JointSpeed.js');
let UserNotificationList = require('./UserNotificationList.js');
let SequenceHandle = require('./SequenceHandle.js');
let UserProfile = require('./UserProfile.js');
let FactoryEvent = require('./FactoryEvent.js');
let MapGroupHandle = require('./MapGroupHandle.js');
let SystemTime = require('./SystemTime.js');
let ControllerBehavior = require('./ControllerBehavior.js');
let ControllerElementHandle = require('./ControllerElementHandle.js');
let Base_ControlMode = require('./Base_ControlMode.js');
let ConstrainedJointAngle = require('./ConstrainedJointAngle.js');
let IPv4Configuration = require('./IPv4Configuration.js');
let MapList = require('./MapList.js');
let MappingInfoNotificationList = require('./MappingInfoNotificationList.js');
let SequenceTaskHandle = require('./SequenceTaskHandle.js');
let GpioAction = require('./GpioAction.js');
let ProtectionZoneInformation = require('./ProtectionZoneInformation.js');
let Gen3GpioPinId = require('./Gen3GpioPinId.js');
let ControllerConfiguration = require('./ControllerConfiguration.js');
let NetworkNotificationList = require('./NetworkNotificationList.js');
let ProtectionZoneList = require('./ProtectionZoneList.js');
let GpioEvent = require('./GpioEvent.js');
let TrajectoryErrorReport = require('./TrajectoryErrorReport.js');
let ArmStateInformation = require('./ArmStateInformation.js');
let ConstrainedPose = require('./ConstrainedPose.js');
let UserProfileList = require('./UserProfileList.js');
let SignalQuality = require('./SignalQuality.js');
let JointAngles = require('./JointAngles.js');
let Base_RotationMatrixRow = require('./Base_RotationMatrixRow.js');
let WrenchLimitation = require('./WrenchLimitation.js');
let SequenceInfoNotification = require('./SequenceInfoNotification.js');
let GpioCommand = require('./GpioCommand.js');
let SequenceTasksPair = require('./SequenceTasksPair.js');
let PreComputedJointTrajectory = require('./PreComputedJointTrajectory.js');
let Base_ServiceVersion = require('./Base_ServiceVersion.js');
let ChangeJointSpeeds = require('./ChangeJointSpeeds.js');
let MapEvent_events = require('./MapEvent_events.js');
let Ssid = require('./Ssid.js');
let ChangeTwist = require('./ChangeTwist.js');
let SafetyEvent = require('./SafetyEvent.js');
let MappingHandle = require('./MappingHandle.js');
let CartesianSpeed = require('./CartesianSpeed.js');
let WifiSecurityType = require('./WifiSecurityType.js');
let ControllerElementState = require('./ControllerElementState.js');
let ChangeWrench = require('./ChangeWrench.js');
let NetworkEvent = require('./NetworkEvent.js');
let AppendActionInformation = require('./AppendActionInformation.js');
let UserEvent = require('./UserEvent.js');
let ConfigurationChangeNotification = require('./ConfigurationChangeNotification.js');
let Query = require('./Query.js');
let CartesianLimitationList = require('./CartesianLimitationList.js');
let BridgeConfig = require('./BridgeConfig.js');
let TrajectoryInfoType = require('./TrajectoryInfoType.js');
let NetworkNotification = require('./NetworkNotification.js');
let CommunicationInterfaceConfiguration = require('./CommunicationInterfaceConfiguration.js');
let Map = require('./Map.js');
let Gripper = require('./Gripper.js');
let WifiInformationList = require('./WifiInformationList.js');
let LimitationType = require('./LimitationType.js');
let Action = require('./Action.js');
let ActionNotification = require('./ActionNotification.js');
let ConstrainedPosition = require('./ConstrainedPosition.js');
let JointAngle = require('./JointAngle.js');
let BridgeResult = require('./BridgeResult.js');
let WaypointList = require('./WaypointList.js');
let ControllerList = require('./ControllerList.js');
let Base_CapSenseMode = require('./Base_CapSenseMode.js');
let SafetyNotificationList = require('./SafetyNotificationList.js');
let MapHandle = require('./MapHandle.js');
let ControllerState = require('./ControllerState.js');
let OperatingModeInformation = require('./OperatingModeInformation.js');
let IPv4Information = require('./IPv4Information.js');
let TwistCommand = require('./TwistCommand.js');
let ControllerNotification_state = require('./ControllerNotification_state.js');
let Pose = require('./Pose.js');
let Action_action_parameters = require('./Action_action_parameters.js');
let Base_SafetyIdentifier = require('./Base_SafetyIdentifier.js');
let ControllerConfigurationList = require('./ControllerConfigurationList.js');
let FullUserProfile = require('./FullUserProfile.js');
let Timeout = require('./Timeout.js');
let ActionType = require('./ActionType.js');
let GpioPinPropertyFlags = require('./GpioPinPropertyFlags.js');
let BackupEvent = require('./BackupEvent.js');
let EmergencyStop = require('./EmergencyStop.js');
let BridgeType = require('./BridgeType.js');
let RobotEventNotificationList = require('./RobotEventNotificationList.js');
let ControllerNotificationList = require('./ControllerNotificationList.js');
let LedState = require('./LedState.js');
let CartesianWaypoint = require('./CartesianWaypoint.js');
let BaseCyclic_CustomData = require('./BaseCyclic_CustomData.js');
let BaseCyclic_ServiceVersion = require('./BaseCyclic_ServiceVersion.js');
let BaseCyclic_Command = require('./BaseCyclic_Command.js');
let ActuatorCustomData = require('./ActuatorCustomData.js');
let ActuatorCommand = require('./ActuatorCommand.js');
let BaseFeedback = require('./BaseFeedback.js');
let ActuatorFeedback = require('./ActuatorFeedback.js');
let BaseCyclic_Feedback = require('./BaseCyclic_Feedback.js');
let SafetyHandle = require('./SafetyHandle.js');
let NotificationType = require('./NotificationType.js');
let NotificationHandle = require('./NotificationHandle.js');
let CountryCode = require('./CountryCode.js');
let UARTSpeed = require('./UARTSpeed.js');
let UARTWordLength = require('./UARTWordLength.js');
let DeviceHandle = require('./DeviceHandle.js');
let NotificationOptions = require('./NotificationOptions.js');
let Timestamp = require('./Timestamp.js');
let SafetyStatusValue = require('./SafetyStatusValue.js');
let Permission = require('./Permission.js');
let UARTParity = require('./UARTParity.js');
let CartesianReferenceFrame = require('./CartesianReferenceFrame.js');
let ArmState = require('./ArmState.js');
let SafetyNotification = require('./SafetyNotification.js');
let UserProfileHandle = require('./UserProfileHandle.js');
let DeviceTypes = require('./DeviceTypes.js');
let UARTConfiguration = require('./UARTConfiguration.js');
let UARTStopBits = require('./UARTStopBits.js');
let Unit = require('./Unit.js');
let Connection = require('./Connection.js');
let Empty = require('./Empty.js');
let CountryCodeIdentifier = require('./CountryCodeIdentifier.js');
let UARTDeviceIdentification = require('./UARTDeviceIdentification.js');
let ControlConfig_Position = require('./ControlConfig_Position.js');
let KinematicLimits = require('./KinematicLimits.js');
let ControlConfig_ControlMode = require('./ControlConfig_ControlMode.js');
let GravityVector = require('./GravityVector.js');
let ControlConfig_ControlModeInformation = require('./ControlConfig_ControlModeInformation.js');
let ControlConfig_ServiceVersion = require('./ControlConfig_ServiceVersion.js');
let PayloadInformation = require('./PayloadInformation.js');
let JointAccelerationSoftLimits = require('./JointAccelerationSoftLimits.js');
let JointSpeedSoftLimits = require('./JointSpeedSoftLimits.js');
let LinearTwist = require('./LinearTwist.js');
let ControlConfig_ControlModeNotification = require('./ControlConfig_ControlModeNotification.js');
let DesiredSpeeds = require('./DesiredSpeeds.js');
let KinematicLimitsList = require('./KinematicLimitsList.js');
let AngularTwist = require('./AngularTwist.js');
let TwistAngularSoftLimit = require('./TwistAngularSoftLimit.js');
let ControlConfig_JointSpeeds = require('./ControlConfig_JointSpeeds.js');
let ToolConfiguration = require('./ToolConfiguration.js');
let ControlConfigurationNotification = require('./ControlConfigurationNotification.js');
let TwistLinearSoftLimit = require('./TwistLinearSoftLimit.js');
let ControlConfigurationEvent = require('./ControlConfigurationEvent.js');
let CartesianTransform = require('./CartesianTransform.js');
let CartesianReferenceFrameInfo = require('./CartesianReferenceFrameInfo.js');
let CapSenseRegister = require('./CapSenseRegister.js');
let CalibrationItem = require('./CalibrationItem.js');
let MACAddress = require('./MACAddress.js');
let SafetyConfigurationList = require('./SafetyConfigurationList.js');
let SafetyConfiguration = require('./SafetyConfiguration.js');
let SerialNumber = require('./SerialNumber.js');
let DeviceConfig_SafetyLimitType = require('./DeviceConfig_SafetyLimitType.js');
let SafetyStatus = require('./SafetyStatus.js');
let SafetyEnable = require('./SafetyEnable.js');
let PartNumber = require('./PartNumber.js');
let DeviceType = require('./DeviceType.js');
let SafetyInformationList = require('./SafetyInformationList.js');
let IPv4Settings = require('./IPv4Settings.js');
let BootloaderVersion = require('./BootloaderVersion.js');
let FirmwareVersion = require('./FirmwareVersion.js');
let DeviceConfig_CapSenseMode = require('./DeviceConfig_CapSenseMode.js');
let CalibrationStatus = require('./CalibrationStatus.js');
let CalibrationResult = require('./CalibrationResult.js');
let SafetyThreshold = require('./SafetyThreshold.js');
let CalibrationElement = require('./CalibrationElement.js');
let PartNumberRevision = require('./PartNumberRevision.js');
let DeviceConfig_ServiceVersion = require('./DeviceConfig_ServiceVersion.js');
let DeviceConfig_CapSenseConfig = require('./DeviceConfig_CapSenseConfig.js');
let CalibrationParameter = require('./CalibrationParameter.js');
let CalibrationParameter_value = require('./CalibrationParameter_value.js');
let RunMode = require('./RunMode.js');
let Calibration = require('./Calibration.js');
let PowerOnSelfTestResult = require('./PowerOnSelfTestResult.js');
let RunModes = require('./RunModes.js');
let RebootRqst = require('./RebootRqst.js');
let SafetyInformation = require('./SafetyInformation.js');
let ModelNumber = require('./ModelNumber.js');
let DeviceHandles = require('./DeviceHandles.js');
let DeviceManager_ServiceVersion = require('./DeviceManager_ServiceVersion.js');
let RobotiqGripperStatusFlags = require('./RobotiqGripperStatusFlags.js');
let GripperConfig_SafetyIdentifier = require('./GripperConfig_SafetyIdentifier.js');
let GripperCyclic_Feedback = require('./GripperCyclic_Feedback.js');
let MotorCommand = require('./MotorCommand.js');
let CustomDataUnit = require('./CustomDataUnit.js');
let GripperCyclic_Command = require('./GripperCyclic_Command.js');
let GripperCyclic_CustomData = require('./GripperCyclic_CustomData.js');
let GripperCyclic_MessageId = require('./GripperCyclic_MessageId.js');
let GripperCyclic_ServiceVersion = require('./GripperCyclic_ServiceVersion.js');
let MotorFeedback = require('./MotorFeedback.js');
let I2CDeviceAddressing = require('./I2CDeviceAddressing.js');
let EthernetDeviceIdentification = require('./EthernetDeviceIdentification.js');
let I2CReadRegisterParameter = require('./I2CReadRegisterParameter.js');
let GPIOState = require('./GPIOState.js');
let I2CDeviceIdentification = require('./I2CDeviceIdentification.js');
let I2CWriteRegisterParameter = require('./I2CWriteRegisterParameter.js');
let InterconnectConfig_ServiceVersion = require('./InterconnectConfig_ServiceVersion.js');
let EthernetSpeed = require('./EthernetSpeed.js');
let GPIOPull = require('./GPIOPull.js');
let GPIOValue = require('./GPIOValue.js');
let EthernetDevice = require('./EthernetDevice.js');
let I2CConfiguration = require('./I2CConfiguration.js');
let I2CDevice = require('./I2CDevice.js');
let GPIOIdentifier = require('./GPIOIdentifier.js');
let I2CWriteParameter = require('./I2CWriteParameter.js');
let EthernetDuplex = require('./EthernetDuplex.js');
let InterconnectConfig_GPIOConfiguration = require('./InterconnectConfig_GPIOConfiguration.js');
let UARTPortId = require('./UARTPortId.js');
let I2CRegisterAddressSize = require('./I2CRegisterAddressSize.js');
let EthernetConfiguration = require('./EthernetConfiguration.js');
let I2CReadParameter = require('./I2CReadParameter.js');
let GPIOIdentification = require('./GPIOIdentification.js');
let GPIOMode = require('./GPIOMode.js');
let InterconnectConfig_SafetyIdentifier = require('./InterconnectConfig_SafetyIdentifier.js');
let I2CData = require('./I2CData.js');
let I2CMode = require('./I2CMode.js');
let InterconnectCyclic_ServiceVersion = require('./InterconnectCyclic_ServiceVersion.js');
let InterconnectCyclic_Feedback_tool_feedback = require('./InterconnectCyclic_Feedback_tool_feedback.js');
let InterconnectCyclic_MessageId = require('./InterconnectCyclic_MessageId.js');
let InterconnectCyclic_CustomData = require('./InterconnectCyclic_CustomData.js');
let InterconnectCyclic_Feedback = require('./InterconnectCyclic_Feedback.js');
let InterconnectCyclic_Command = require('./InterconnectCyclic_Command.js');
let InterconnectCyclic_Command_tool_command = require('./InterconnectCyclic_Command_tool_command.js');
let InterconnectCyclic_CustomData_tool_customData = require('./InterconnectCyclic_CustomData_tool_customData.js');
let BaseType = require('./BaseType.js');
let EndEffectorType = require('./EndEffectorType.js');
let CompleteProductConfiguration = require('./CompleteProductConfiguration.js');
let InterfaceModuleType = require('./InterfaceModuleType.js');
let ProductConfigurationEndEffectorType = require('./ProductConfigurationEndEffectorType.js');
let ModelId = require('./ModelId.js');
let VisionModuleType = require('./VisionModuleType.js');
let ArmLaterality = require('./ArmLaterality.js');
let WristType = require('./WristType.js');
let OptionValue = require('./OptionValue.js');
let IntrinsicProfileIdentifier = require('./IntrinsicProfileIdentifier.js');
let BitRate = require('./BitRate.js');
let TranslationVector = require('./TranslationVector.js');
let ExtrinsicParameters = require('./ExtrinsicParameters.js');
let OptionIdentifier = require('./OptionIdentifier.js');
let SensorFocusAction = require('./SensorFocusAction.js');
let IntrinsicParameters = require('./IntrinsicParameters.js');
let VisionConfig_ServiceVersion = require('./VisionConfig_ServiceVersion.js');
let VisionEvent = require('./VisionEvent.js');
let VisionConfig_RotationMatrix = require('./VisionConfig_RotationMatrix.js');
let Sensor = require('./Sensor.js');
let Option = require('./Option.js');
let Resolution = require('./Resolution.js');
let VisionNotification = require('./VisionNotification.js');
let SensorFocusAction_action_parameters = require('./SensorFocusAction_action_parameters.js');
let FocusAction = require('./FocusAction.js');
let DistortionCoefficients = require('./DistortionCoefficients.js');
let FrameRate = require('./FrameRate.js');
let OptionInformation = require('./OptionInformation.js');
let VisionConfig_RotationMatrixRow = require('./VisionConfig_RotationMatrixRow.js');
let SensorSettings = require('./SensorSettings.js');
let SensorIdentifier = require('./SensorIdentifier.js');
let FocusPoint = require('./FocusPoint.js');
let ManualFocus = require('./ManualFocus.js');
let FollowCartesianTrajectoryAction = require('./FollowCartesianTrajectoryAction.js');
let FollowCartesianTrajectoryFeedback = require('./FollowCartesianTrajectoryFeedback.js');
let FollowCartesianTrajectoryResult = require('./FollowCartesianTrajectoryResult.js');
let FollowCartesianTrajectoryActionResult = require('./FollowCartesianTrajectoryActionResult.js');
let FollowCartesianTrajectoryActionFeedback = require('./FollowCartesianTrajectoryActionFeedback.js');
let FollowCartesianTrajectoryActionGoal = require('./FollowCartesianTrajectoryActionGoal.js');
let FollowCartesianTrajectoryGoal = require('./FollowCartesianTrajectoryGoal.js');

module.exports = {
  KortexError: KortexError,
  ApiOptions: ApiOptions,
  SubErrorCodes: SubErrorCodes,
  ErrorCodes: ErrorCodes,
  PositionCommand: PositionCommand,
  SafetyIdentifierBankA: SafetyIdentifierBankA,
  CoggingFeedforwardModeInformation: CoggingFeedforwardModeInformation,
  ActuatorConfig_ControlMode: ActuatorConfig_ControlMode,
  ControlLoop: ControlLoop,
  CommandModeInformation: CommandModeInformation,
  TorqueOffset: TorqueOffset,
  CoggingFeedforwardMode: CoggingFeedforwardMode,
  FrequencyResponse: FrequencyResponse,
  ActuatorConfig_ControlModeInformation: ActuatorConfig_ControlModeInformation,
  EncoderDerivativeParameters: EncoderDerivativeParameters,
  ActuatorConfig_SafetyLimitType: ActuatorConfig_SafetyLimitType,
  ControlLoopParameters: ControlLoopParameters,
  CustomDataIndex: CustomDataIndex,
  VectorDriveParameters: VectorDriveParameters,
  CommandMode: CommandMode,
  StepResponse: StepResponse,
  AxisPosition: AxisPosition,
  CustomDataSelection: CustomDataSelection,
  ControlLoopSelection: ControlLoopSelection,
  Servoing: Servoing,
  LoopSelection: LoopSelection,
  AxisOffsets: AxisOffsets,
  TorqueCalibration: TorqueCalibration,
  ActuatorConfig_ServiceVersion: ActuatorConfig_ServiceVersion,
  RampResponse: RampResponse,
  ActuatorCyclic_ServiceVersion: ActuatorCyclic_ServiceVersion,
  ActuatorCyclic_Feedback: ActuatorCyclic_Feedback,
  CommandFlags: CommandFlags,
  ActuatorCyclic_Command: ActuatorCyclic_Command,
  ActuatorCyclic_MessageId: ActuatorCyclic_MessageId,
  ActuatorCyclic_CustomData: ActuatorCyclic_CustomData,
  StatusFlags: StatusFlags,
  ControllerHandle: ControllerHandle,
  Sequence: Sequence,
  ArmStateNotification: ArmStateNotification,
  NetworkType: NetworkType,
  GpioPinConfiguration: GpioPinConfiguration,
  WifiConfigurationList: WifiConfigurationList,
  ServoingModeInformation: ServoingModeInformation,
  CartesianLimitation: CartesianLimitation,
  FirmwareBundleVersions: FirmwareBundleVersions,
  ConfigurationNotificationEvent: ConfigurationNotificationEvent,
  MapGroup: MapGroup,
  SoundType: SoundType,
  TrajectoryErrorIdentifier: TrajectoryErrorIdentifier,
  TrajectoryInfo: TrajectoryInfo,
  JointTrajectoryConstraintType: JointTrajectoryConstraintType,
  TransformationMatrix: TransformationMatrix,
  ControllerElementEventType: ControllerElementEventType,
  WrenchCommand: WrenchCommand,
  ControlModeNotificationList: ControlModeNotificationList,
  WifiConfiguration: WifiConfiguration,
  OperatingMode: OperatingMode,
  Base_ControlModeInformation: Base_ControlModeInformation,
  FirmwareComponentVersion: FirmwareComponentVersion,
  PreComputedJointTrajectoryElement: PreComputedJointTrajectoryElement,
  JointTorque: JointTorque,
  WifiInformation: WifiInformation,
  JointNavigationDirection: JointNavigationDirection,
  ActionExecutionState: ActionExecutionState,
  MapElement: MapElement,
  ControllerEventType: ControllerEventType,
  JointsLimitationsList: JointsLimitationsList,
  UserNotification: UserNotification,
  ProtectionZoneNotification: ProtectionZoneNotification,
  MapGroupList: MapGroupList,
  SequenceInformation: SequenceInformation,
  SequenceTasksConfiguration: SequenceTasksConfiguration,
  ActionList: ActionList,
  ServoingMode: ServoingMode,
  TransformationRow: TransformationRow,
  JointLimitation: JointLimitation,
  GpioConfigurationList: GpioConfigurationList,
  SequenceTaskConfiguration: SequenceTaskConfiguration,
  NavigationDirection: NavigationDirection,
  ActuatorInformation: ActuatorInformation,
  ServoingModeNotification: ServoingModeNotification,
  ConfigurationChangeNotificationList: ConfigurationChangeNotificationList,
  ZoneShape: ZoneShape,
  SequenceTasks: SequenceTasks,
  MappingInfoNotification: MappingInfoNotification,
  ProtectionZoneHandle: ProtectionZoneHandle,
  OperatingModeNotificationList: OperatingModeNotificationList,
  BridgePortConfig: BridgePortConfig,
  SnapshotType: SnapshotType,
  SequenceInfoNotificationList: SequenceInfoNotificationList,
  ControllerInputType: ControllerInputType,
  TrajectoryErrorElement: TrajectoryErrorElement,
  Orientation: Orientation,
  FactoryNotification: FactoryNotification,
  RequestedActionType: RequestedActionType,
  ActionEvent: ActionEvent,
  GripperCommand: GripperCommand,
  EventIdSequenceInfoNotification: EventIdSequenceInfoNotification,
  SwitchControlMapping: SwitchControlMapping,
  Base_Position: Base_Position,
  TwistLimitation: TwistLimitation,
  RobotEventNotification: RobotEventNotification,
  Base_GpioConfiguration: Base_GpioConfiguration,
  ActionNotificationList: ActionNotificationList,
  UserList: UserList,
  OperatingModeNotification: OperatingModeNotification,
  WifiEncryptionType: WifiEncryptionType,
  MappingList: MappingList,
  ControllerNotification: ControllerNotification,
  Wrench: Wrench,
  AngularWaypoint: AngularWaypoint,
  KinematicTrajectoryConstraints: KinematicTrajectoryConstraints,
  ConfigurationChangeNotification_configuration_change: ConfigurationChangeNotification_configuration_change,
  TrajectoryErrorType: TrajectoryErrorType,
  Base_RotationMatrix: Base_RotationMatrix,
  Base_Stop: Base_Stop,
  WaypointValidationReport: WaypointValidationReport,
  WrenchMode: WrenchMode,
  Xbox360DigitalInputIdentifier: Xbox360DigitalInputIdentifier,
  FullIPv4Configuration: FullIPv4Configuration,
  Faults: Faults,
  PasswordChange: PasswordChange,
  Base_ControlModeNotification: Base_ControlModeNotification,
  NetworkHandle: NetworkHandle,
  Waypoint: Waypoint,
  TrajectoryContinuityMode: TrajectoryContinuityMode,
  WristDigitalInputIdentifier: WristDigitalInputIdentifier,
  AdvancedSequenceHandle: AdvancedSequenceHandle,
  BridgeIdentifier: BridgeIdentifier,
  GripperRequest: GripperRequest,
  SequenceTask: SequenceTask,
  ShapeType: ShapeType,
  ControllerType: ControllerType,
  Xbox360AnalogInputIdentifier: Xbox360AnalogInputIdentifier,
  CartesianTrajectoryConstraint_type: CartesianTrajectoryConstraint_type,
  IKData: IKData,
  Snapshot: Snapshot,
  Base_JointSpeeds: Base_JointSpeeds,
  MapEvent: MapEvent,
  ProtectionZone: ProtectionZone,
  ControllerElementHandle_identifier: ControllerElementHandle_identifier,
  JointTorques: JointTorques,
  ConstrainedOrientation: ConstrainedOrientation,
  CartesianTrajectoryConstraint: CartesianTrajectoryConstraint,
  ProtectionZoneNotificationList: ProtectionZoneNotificationList,
  ProtectionZoneEvent: ProtectionZoneEvent,
  Twist: Twist,
  ControllerEvent: ControllerEvent,
  ControllerConfigurationMode: ControllerConfigurationMode,
  GripperMode: GripperMode,
  AdmittanceMode: AdmittanceMode,
  Point: Point,
  SequenceTasksRange: SequenceTasksRange,
  Base_CapSenseConfig: Base_CapSenseConfig,
  BridgeList: BridgeList,
  ActionHandle: ActionHandle,
  BridgeStatus: BridgeStatus,
  ConstrainedJointAngles: ConstrainedJointAngles,
  Mapping: Mapping,
  Finger: Finger,
  GpioBehavior: GpioBehavior,
  ActivateMapHandle: ActivateMapHandle,
  RobotEvent: RobotEvent,
  Delay: Delay,
  SequenceList: SequenceList,
  Waypoint_type_of_waypoint: Waypoint_type_of_waypoint,
  Admittance: Admittance,
  JointTrajectoryConstraint: JointTrajectoryConstraint,
  ServoingModeNotificationList: ServoingModeNotificationList,
  JointSpeed: JointSpeed,
  UserNotificationList: UserNotificationList,
  SequenceHandle: SequenceHandle,
  UserProfile: UserProfile,
  FactoryEvent: FactoryEvent,
  MapGroupHandle: MapGroupHandle,
  SystemTime: SystemTime,
  ControllerBehavior: ControllerBehavior,
  ControllerElementHandle: ControllerElementHandle,
  Base_ControlMode: Base_ControlMode,
  ConstrainedJointAngle: ConstrainedJointAngle,
  IPv4Configuration: IPv4Configuration,
  MapList: MapList,
  MappingInfoNotificationList: MappingInfoNotificationList,
  SequenceTaskHandle: SequenceTaskHandle,
  GpioAction: GpioAction,
  ProtectionZoneInformation: ProtectionZoneInformation,
  Gen3GpioPinId: Gen3GpioPinId,
  ControllerConfiguration: ControllerConfiguration,
  NetworkNotificationList: NetworkNotificationList,
  ProtectionZoneList: ProtectionZoneList,
  GpioEvent: GpioEvent,
  TrajectoryErrorReport: TrajectoryErrorReport,
  ArmStateInformation: ArmStateInformation,
  ConstrainedPose: ConstrainedPose,
  UserProfileList: UserProfileList,
  SignalQuality: SignalQuality,
  JointAngles: JointAngles,
  Base_RotationMatrixRow: Base_RotationMatrixRow,
  WrenchLimitation: WrenchLimitation,
  SequenceInfoNotification: SequenceInfoNotification,
  GpioCommand: GpioCommand,
  SequenceTasksPair: SequenceTasksPair,
  PreComputedJointTrajectory: PreComputedJointTrajectory,
  Base_ServiceVersion: Base_ServiceVersion,
  ChangeJointSpeeds: ChangeJointSpeeds,
  MapEvent_events: MapEvent_events,
  Ssid: Ssid,
  ChangeTwist: ChangeTwist,
  SafetyEvent: SafetyEvent,
  MappingHandle: MappingHandle,
  CartesianSpeed: CartesianSpeed,
  WifiSecurityType: WifiSecurityType,
  ControllerElementState: ControllerElementState,
  ChangeWrench: ChangeWrench,
  NetworkEvent: NetworkEvent,
  AppendActionInformation: AppendActionInformation,
  UserEvent: UserEvent,
  ConfigurationChangeNotification: ConfigurationChangeNotification,
  Query: Query,
  CartesianLimitationList: CartesianLimitationList,
  BridgeConfig: BridgeConfig,
  TrajectoryInfoType: TrajectoryInfoType,
  NetworkNotification: NetworkNotification,
  CommunicationInterfaceConfiguration: CommunicationInterfaceConfiguration,
  Map: Map,
  Gripper: Gripper,
  WifiInformationList: WifiInformationList,
  LimitationType: LimitationType,
  Action: Action,
  ActionNotification: ActionNotification,
  ConstrainedPosition: ConstrainedPosition,
  JointAngle: JointAngle,
  BridgeResult: BridgeResult,
  WaypointList: WaypointList,
  ControllerList: ControllerList,
  Base_CapSenseMode: Base_CapSenseMode,
  SafetyNotificationList: SafetyNotificationList,
  MapHandle: MapHandle,
  ControllerState: ControllerState,
  OperatingModeInformation: OperatingModeInformation,
  IPv4Information: IPv4Information,
  TwistCommand: TwistCommand,
  ControllerNotification_state: ControllerNotification_state,
  Pose: Pose,
  Action_action_parameters: Action_action_parameters,
  Base_SafetyIdentifier: Base_SafetyIdentifier,
  ControllerConfigurationList: ControllerConfigurationList,
  FullUserProfile: FullUserProfile,
  Timeout: Timeout,
  ActionType: ActionType,
  GpioPinPropertyFlags: GpioPinPropertyFlags,
  BackupEvent: BackupEvent,
  EmergencyStop: EmergencyStop,
  BridgeType: BridgeType,
  RobotEventNotificationList: RobotEventNotificationList,
  ControllerNotificationList: ControllerNotificationList,
  LedState: LedState,
  CartesianWaypoint: CartesianWaypoint,
  BaseCyclic_CustomData: BaseCyclic_CustomData,
  BaseCyclic_ServiceVersion: BaseCyclic_ServiceVersion,
  BaseCyclic_Command: BaseCyclic_Command,
  ActuatorCustomData: ActuatorCustomData,
  ActuatorCommand: ActuatorCommand,
  BaseFeedback: BaseFeedback,
  ActuatorFeedback: ActuatorFeedback,
  BaseCyclic_Feedback: BaseCyclic_Feedback,
  SafetyHandle: SafetyHandle,
  NotificationType: NotificationType,
  NotificationHandle: NotificationHandle,
  CountryCode: CountryCode,
  UARTSpeed: UARTSpeed,
  UARTWordLength: UARTWordLength,
  DeviceHandle: DeviceHandle,
  NotificationOptions: NotificationOptions,
  Timestamp: Timestamp,
  SafetyStatusValue: SafetyStatusValue,
  Permission: Permission,
  UARTParity: UARTParity,
  CartesianReferenceFrame: CartesianReferenceFrame,
  ArmState: ArmState,
  SafetyNotification: SafetyNotification,
  UserProfileHandle: UserProfileHandle,
  DeviceTypes: DeviceTypes,
  UARTConfiguration: UARTConfiguration,
  UARTStopBits: UARTStopBits,
  Unit: Unit,
  Connection: Connection,
  Empty: Empty,
  CountryCodeIdentifier: CountryCodeIdentifier,
  UARTDeviceIdentification: UARTDeviceIdentification,
  ControlConfig_Position: ControlConfig_Position,
  KinematicLimits: KinematicLimits,
  ControlConfig_ControlMode: ControlConfig_ControlMode,
  GravityVector: GravityVector,
  ControlConfig_ControlModeInformation: ControlConfig_ControlModeInformation,
  ControlConfig_ServiceVersion: ControlConfig_ServiceVersion,
  PayloadInformation: PayloadInformation,
  JointAccelerationSoftLimits: JointAccelerationSoftLimits,
  JointSpeedSoftLimits: JointSpeedSoftLimits,
  LinearTwist: LinearTwist,
  ControlConfig_ControlModeNotification: ControlConfig_ControlModeNotification,
  DesiredSpeeds: DesiredSpeeds,
  KinematicLimitsList: KinematicLimitsList,
  AngularTwist: AngularTwist,
  TwistAngularSoftLimit: TwistAngularSoftLimit,
  ControlConfig_JointSpeeds: ControlConfig_JointSpeeds,
  ToolConfiguration: ToolConfiguration,
  ControlConfigurationNotification: ControlConfigurationNotification,
  TwistLinearSoftLimit: TwistLinearSoftLimit,
  ControlConfigurationEvent: ControlConfigurationEvent,
  CartesianTransform: CartesianTransform,
  CartesianReferenceFrameInfo: CartesianReferenceFrameInfo,
  CapSenseRegister: CapSenseRegister,
  CalibrationItem: CalibrationItem,
  MACAddress: MACAddress,
  SafetyConfigurationList: SafetyConfigurationList,
  SafetyConfiguration: SafetyConfiguration,
  SerialNumber: SerialNumber,
  DeviceConfig_SafetyLimitType: DeviceConfig_SafetyLimitType,
  SafetyStatus: SafetyStatus,
  SafetyEnable: SafetyEnable,
  PartNumber: PartNumber,
  DeviceType: DeviceType,
  SafetyInformationList: SafetyInformationList,
  IPv4Settings: IPv4Settings,
  BootloaderVersion: BootloaderVersion,
  FirmwareVersion: FirmwareVersion,
  DeviceConfig_CapSenseMode: DeviceConfig_CapSenseMode,
  CalibrationStatus: CalibrationStatus,
  CalibrationResult: CalibrationResult,
  SafetyThreshold: SafetyThreshold,
  CalibrationElement: CalibrationElement,
  PartNumberRevision: PartNumberRevision,
  DeviceConfig_ServiceVersion: DeviceConfig_ServiceVersion,
  DeviceConfig_CapSenseConfig: DeviceConfig_CapSenseConfig,
  CalibrationParameter: CalibrationParameter,
  CalibrationParameter_value: CalibrationParameter_value,
  RunMode: RunMode,
  Calibration: Calibration,
  PowerOnSelfTestResult: PowerOnSelfTestResult,
  RunModes: RunModes,
  RebootRqst: RebootRqst,
  SafetyInformation: SafetyInformation,
  ModelNumber: ModelNumber,
  DeviceHandles: DeviceHandles,
  DeviceManager_ServiceVersion: DeviceManager_ServiceVersion,
  RobotiqGripperStatusFlags: RobotiqGripperStatusFlags,
  GripperConfig_SafetyIdentifier: GripperConfig_SafetyIdentifier,
  GripperCyclic_Feedback: GripperCyclic_Feedback,
  MotorCommand: MotorCommand,
  CustomDataUnit: CustomDataUnit,
  GripperCyclic_Command: GripperCyclic_Command,
  GripperCyclic_CustomData: GripperCyclic_CustomData,
  GripperCyclic_MessageId: GripperCyclic_MessageId,
  GripperCyclic_ServiceVersion: GripperCyclic_ServiceVersion,
  MotorFeedback: MotorFeedback,
  I2CDeviceAddressing: I2CDeviceAddressing,
  EthernetDeviceIdentification: EthernetDeviceIdentification,
  I2CReadRegisterParameter: I2CReadRegisterParameter,
  GPIOState: GPIOState,
  I2CDeviceIdentification: I2CDeviceIdentification,
  I2CWriteRegisterParameter: I2CWriteRegisterParameter,
  InterconnectConfig_ServiceVersion: InterconnectConfig_ServiceVersion,
  EthernetSpeed: EthernetSpeed,
  GPIOPull: GPIOPull,
  GPIOValue: GPIOValue,
  EthernetDevice: EthernetDevice,
  I2CConfiguration: I2CConfiguration,
  I2CDevice: I2CDevice,
  GPIOIdentifier: GPIOIdentifier,
  I2CWriteParameter: I2CWriteParameter,
  EthernetDuplex: EthernetDuplex,
  InterconnectConfig_GPIOConfiguration: InterconnectConfig_GPIOConfiguration,
  UARTPortId: UARTPortId,
  I2CRegisterAddressSize: I2CRegisterAddressSize,
  EthernetConfiguration: EthernetConfiguration,
  I2CReadParameter: I2CReadParameter,
  GPIOIdentification: GPIOIdentification,
  GPIOMode: GPIOMode,
  InterconnectConfig_SafetyIdentifier: InterconnectConfig_SafetyIdentifier,
  I2CData: I2CData,
  I2CMode: I2CMode,
  InterconnectCyclic_ServiceVersion: InterconnectCyclic_ServiceVersion,
  InterconnectCyclic_Feedback_tool_feedback: InterconnectCyclic_Feedback_tool_feedback,
  InterconnectCyclic_MessageId: InterconnectCyclic_MessageId,
  InterconnectCyclic_CustomData: InterconnectCyclic_CustomData,
  InterconnectCyclic_Feedback: InterconnectCyclic_Feedback,
  InterconnectCyclic_Command: InterconnectCyclic_Command,
  InterconnectCyclic_Command_tool_command: InterconnectCyclic_Command_tool_command,
  InterconnectCyclic_CustomData_tool_customData: InterconnectCyclic_CustomData_tool_customData,
  BaseType: BaseType,
  EndEffectorType: EndEffectorType,
  CompleteProductConfiguration: CompleteProductConfiguration,
  InterfaceModuleType: InterfaceModuleType,
  ProductConfigurationEndEffectorType: ProductConfigurationEndEffectorType,
  ModelId: ModelId,
  VisionModuleType: VisionModuleType,
  ArmLaterality: ArmLaterality,
  WristType: WristType,
  OptionValue: OptionValue,
  IntrinsicProfileIdentifier: IntrinsicProfileIdentifier,
  BitRate: BitRate,
  TranslationVector: TranslationVector,
  ExtrinsicParameters: ExtrinsicParameters,
  OptionIdentifier: OptionIdentifier,
  SensorFocusAction: SensorFocusAction,
  IntrinsicParameters: IntrinsicParameters,
  VisionConfig_ServiceVersion: VisionConfig_ServiceVersion,
  VisionEvent: VisionEvent,
  VisionConfig_RotationMatrix: VisionConfig_RotationMatrix,
  Sensor: Sensor,
  Option: Option,
  Resolution: Resolution,
  VisionNotification: VisionNotification,
  SensorFocusAction_action_parameters: SensorFocusAction_action_parameters,
  FocusAction: FocusAction,
  DistortionCoefficients: DistortionCoefficients,
  FrameRate: FrameRate,
  OptionInformation: OptionInformation,
  VisionConfig_RotationMatrixRow: VisionConfig_RotationMatrixRow,
  SensorSettings: SensorSettings,
  SensorIdentifier: SensorIdentifier,
  FocusPoint: FocusPoint,
  ManualFocus: ManualFocus,
  FollowCartesianTrajectoryAction: FollowCartesianTrajectoryAction,
  FollowCartesianTrajectoryFeedback: FollowCartesianTrajectoryFeedback,
  FollowCartesianTrajectoryResult: FollowCartesianTrajectoryResult,
  FollowCartesianTrajectoryActionResult: FollowCartesianTrajectoryActionResult,
  FollowCartesianTrajectoryActionFeedback: FollowCartesianTrajectoryActionFeedback,
  FollowCartesianTrajectoryActionGoal: FollowCartesianTrajectoryActionGoal,
  FollowCartesianTrajectoryGoal: FollowCartesianTrajectoryGoal,
};
