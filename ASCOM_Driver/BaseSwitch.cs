/*
 * SwitchDriver.cs
 * Copyright (C) 2022 - Present, Julien Lecomte - All Rights Reserved
 * Licensed under the MIT License. See the accompanying LICENSE file for terms.
 */

namespace ASCOM.Astroswell
{
    internal class BaseSwitch
    {
        public string Name { get; set; }
        public string Description { get; set; }
        public string SetCommand { get; set; }
        public string GetCommand { get; set; }
    }

    internal class ReadonlySwitch : BaseSwitch
    {
    }

    internal class ToggleSwitch : BaseSwitch
    {
    }

    internal class RangeSwitch : BaseSwitch
    {
        public int Min { get; set; }
        public int Max { get; set; }
        public int Step { get; set; }
    }
}