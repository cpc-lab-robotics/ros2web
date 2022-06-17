import Table from "./table";
import ServiceCall from "./service-call";
import Blank from "./blank";
import ParamConfig from "./param-config";
import Stack from "./stack";
import Button from "./button";
import Joystick from "./joystick";
import Plot from "./plot";


export const WidgetClass: { [key: string]: any } = {};

WidgetClass["Table"] = Table;
WidgetClass["ServiceCall"] = ServiceCall;
WidgetClass["Blank"] = Blank;
WidgetClass["ParamConfig"] = ParamConfig;
WidgetClass["Stack"] = Stack;
WidgetClass["Button"] = Button;
WidgetClass["Joystick"] = Joystick;
WidgetClass["Plot"] = Plot;

