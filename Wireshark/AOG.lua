-- Declare the AOGProtocol dissector
AOGProtocol_proto = Proto("AgOpenGPS", "AgOpenGPS Protocol")

local MajorPGNs = {
    [0x7F] = "Steer module",
    [0xFE] = "From AutoSteer"
}
local MinorPGNs = {
    [0xFE] = "Speed",
    [0xFC] = "Steer Settings"
}

local CANDebugState = {
    [0] = "Disable diagnostics",
    [1] = "Enable diagnostics",
    [2] = "Disable filters",
    [3] = "Enable filters"
}

local AOGFields = {
    H1 = ProtoField.uint8("AOGProtocol.Hello1", "Hello1", base.HEX),
    H2 = ProtoField.uint8("AOGProtocol.Hello2", "Hello2", base.HEX),
    PGN = ProtoField.uint8("AOGProtocol.PGN", "PGN", base.HEX, MajorPGNs),
    PGN2 = ProtoField.uint8("AOGProtocol.PGN2", "PGN2", base.HEX, MinorPGNs),

    pivotLat = ProtoField.uint16("CorrectedGPS.pivotLat", "pivotLat", base.DEC),
    pivotLon = ProtoField.uint16("CorrectedGPS.pivotLon", "pivotLon", base.DEC),
    fixHeading = ProtoField.uint16("CorrectedGPS.fixHeading", "fixHeading", base.DEC),
    pivotAlt = ProtoField.uint16("CorrectedGPS.pivotAlt", "pivotAlt", base.DEC),

    fCANDebugState = ProtoField.uint8("AOGProtocol.CANDebugState", "CANDebugState", base.DEC, CANDebugState),

    Speed = ProtoField.uint16("SteerData.Speed", "Speed", base.DEC),

    ssKp = ProtoField.uint16("SteerSettings.kp", "kp", base.DEC),
    ssHighPWM = ProtoField.uint16("SteerSettings.HighPWM", "HighPWM", base.DEC),
    ssLowPWM = ProtoField.uint16("SteerSettings.LowPWM", "LowPWM", base.DEC),
    ssMinPWM = ProtoField.uint16("SteerSettings.MinPWM", "MinPWM", base.DEC),
    ssSteerSensorCounts = ProtoField.uint16("SteerSettings.ssCounts", "Steer Sensor Counts", base.DEC),
    sswasOffset = ProtoField.uint16("SteerSettings.wasOffset", "WAS Offset", base.DEC),
    ssAckermanFix = ProtoField.uint16("SteerSettings.AckermanFix", "Ackerman Fix", base.DEC),
    
    genericIP = ProtoField.unit32("GenericIP.IPv4","IPv4", base.DEC),
    genericSubnet = ProtoField.unit32("GenericIP.IPSubnet","Subnet", base.DEC)
}


FixQuality = {
    [0] = "Invalid",
    [1] = "GPS fix (SPS)",
    [2] = "DGPS fix",
    [3] = "PPS fix",
    [4] = "Real Time Kinematic",
    [5] = "Float RTK",
    [6] = "estimated (dead reckoning) (2.3 feature)",
    [7] = "Manual input mode",
    [8] = "Simulation mode"
}

-- local function lookupFixQuality(value)
--     print("!")
--     return FixQuality[value] or "Unknown"
-- end

local function isBitSet(number, bitPosition)
    local bitMask = 2 ^ (bitPosition - 1)
    return number % (bitMask + bitMask) >= bitMask
end

local PandaFields = {"MessageType", "fixTime", "latitude", "LatNS", "longitude", "LonEW", "FixQuality", "numSats",
                     "HDOP", "Altitude", "AgeDGPS", "SpeedKnots", "imuHeading", "imuRoll", "imuYawRate", "StarChecksum"}
local PandaFieldsProto = {}

for _, fieldName in ipairs(PandaFields) do
    if fieldName == "FixQuality" then
        field = ProtoField.int8("PANDA." .. fieldName, fieldName, base.ASCII, FixQuality)
    else
        field = ProtoField.string("PANDA." .. fieldName, fieldName, base.ASCII)
    end
    table.insert(PandaFieldsProto, field)
end

-- Merge all field tables into a single table
local allFields = {}
for key, value in pairs(AOGFields) do
    table.insert(allFields, value)
    -- allFields[key] = value
end

for key, value in pairs(PandaFieldsProto) do
    table.insert(allFields, value)
end

local eInt16, encodedAngle
-- and register those fields
AOGProtocol_proto.fields = allFields

function AOGProtocol_proto.dissector(buffer, pinfo, tree)

    if buffer:len() < 4 then
        return
    end

    local subtree = tree:add(AOGProtocol_proto, buffer(), "AOG Data")

    local byte1 = buffer(0, 1):uint()
    local byte2 = buffer(1, 1):uint()
    local MajorPGN = buffer(2, 1):uint()
    local MinorPGN = buffer(3, 1):uint()
    encodedAngle = buffer(5, 4):le_uint()
    print(string.format("%x", encodedAngle))

    if byte1 == 0x24 and byte2 == 0x50 then -- PANDA
        local PandaString = buffer(0):string()
        local values = {}
        for value in PandaString:gmatch("[^,]+") do
            table.insert(values, value)
        end
        -- rewrite latitude
        local mul
        local degrees = tonumber(string.sub(values[3], 1, 2))
        local minutes = tonumber(string.sub(values[3], 3)) or 0
        if values[4] == "S" then
            values[3] = degrees + (minutes / 60) * -1
        else
            values[3] = degrees + (minutes / 60)
        end
        -- rewrite longitude
        local degrees = tonumber(string.sub(values[5], 1, 2))
        local minutes = tonumber(string.sub(values[5], 3)) or 0
        if values[6] == "W" then
            values[5] = degrees + (minutes / 60) * -1
        else
            values[5] = degrees + (minutes / 60)
        end
        for i, value in ipairs(values) do
            subtree:add(PandaFieldsProto[i], values[i])
        end
        pinfo.cols.info = "AOG Location response"

    else
        if byte1 == 0x80 and byte2 == 0x81 then -- we're into PGNs from AOG now
            if MajorPGN == 0x7f then -- steer module
                if MinorPGN == 0xaa then -- 170
                    pinfo.cols.info = "CANBUS manufacturer change to id:" .. buffer(5, 1)
                end
                if MinorPGN == 0xab then -- 171
                    pinfo.cols.info = "CANBUS query manufacturer response: " .. buffer(5, 1)
                end
                if MinorPGN == 0xac then -- 172
                    pinfo.cols.info = "CANBUS Set logging state"
                    subtree:add(AOGFields.fCANDebugState, buffer(5, 1))
                end
                if MinorPGN == 0xfe then -- 254
                    pinfo.cols.info = "Steer data"
                end
                if MinorPGN == 0xc8 then -- 200
                    pinfo.cols.info = "Hello from AgIO!"
                end
                if MinorPGN == 0xc9 then -- 201
                    pinfo.cols.info = "Subnet change"
                end
                if MinorPGN == 0xca then -- 202
                    pinfo.cols.info = "Subnet scan request"
                end
                if MinorPGN == 0xcb then -- 203
                    subtree:add(AOGFields.genericIP, buffer(5,4))
                    subtree:add(AOGFields.genericSubnet, buffer(9,3))
                    pinfo.cols.info = "Subnet scan request"
                end
                if MinorPGN == 0xd0 then -- Corrected GPS data
                    pinfo.cols.info = "Corrected GPS data"
                    encodedAngle = buffer(5, 4):le_uint()
                    subtree:add(AOGFields.pivotLat, (encodedAngle * 0.0000001) - 210)
                    encodedAngle = buffer(9, 4):le_uint()
                    subtree:add(AOGFields.pivotLon, (encodedAngle * 0.0000001) - 210)
                    eInt16 = buffer(13, 2):le_uint()
                    subtree:add(AOGFields.fixHeading, (eInt16 / 128))
                    eInt16 = buffer(15, 2):le_uint()
                    subtree:add(AOGFields.pivotAlt, (eInt16 * 0.01))
                end
                if MinorPGN == 0xec then
                    pinfo.cols.info = "Machine settings"
                end
                if MinorPGN == 0xee then
                    pinfo.cols.info = "Machine config"
                end
                if MinorPGN == 0xef then -- machine data
                    pinfo.cols.info = "Machine data"
                end
                if MinorPGN == 0xfb then -- Steer config
                    pinfo.cols.info = "Steer config"
                end
                if MinorPGN == 0xfc then -- steer settings
                    pinfo.cols.info = "Steer settings"
                end
            end

            if MajorPGN == 0x78 then -- From GPS module 120
            end
            if MajorPGN == 0x79 then -- From IMU 121
            end
            -- woah, watch out here, as it's back-checking data[2] rather than data[3], it's a TRAP!
            -- some weird shit going on here, see #403 ReceiveFromLoopBack
            if MajorPGN == 0x7b or MajorPGN == 0x7e or MajorPGN == 0x79 then -- Machine Module 123 or autosteer 126 or IMU 121
                subtree:add(AOGFields.genericIP, buffer(5,4))
                subtree:add(AOGFields.genericSubnet, buffer(9,3))
            end
            if MajorPGN == 0x7C then -- From GPS
            end
            if MajorPGN == 0x7D then -- From IMU
            end
            -- if MajorPGN == 0x7e then -- from autosteer 126
            --     subtree:add(AOGFields.genericIP, buffer(5,4))
            --     subtree:add(AOGFields.genericSubnet, buffer(9,3))
            -- end

            -- subtree:add(AOGFields.H1, buffer(0, 1))
            -- subtree:add(AOGFields.H2, buffer(1, 1))
            -- subtree:add(AOGFields.PGN, PGN)
            -- subtree:add(AOGFields.PGN2, buffer(3, 1))

            if PGN == 0x41 then
                subtree:add(SteerData.Speed, buffer(4, 2))
            end
        end
    end
    -- Set the protocol description in the packet details pane
    pinfo.cols.protocol = AOGProtocol_proto.name

end

-- Register the AOGProtocol dissector
local udp_port = DissectorTable.get("udp.port")
udp_port:add(9999, AOGProtocol_proto)
