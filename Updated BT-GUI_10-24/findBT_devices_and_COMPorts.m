%% Find BT Devices and matching COM Ports in the Windows registry
function BT_devices = findBT_devices_and_COMPorts()
    % FINDBT_DEVICES_AND_COMPORTS Finds Bluetooth devices and their associated COM ports.
    % This function queries the Windows registry to find Bluetooth devices and their COM
    % ports, then matches the devices to a predefined list of device names.

    % Command to query the registry for Bluetooth devices and friendly names
    cmd = 'REG QUERY HKEY_LOCAL_MACHINE\SYSTEM\CurrentControlSet\Enum\ /s /f "FriendlyName" /t "REG_SZ"';
    
    % Execute the command and capture the output in largeRegistryString
    [~, largeRegistryString] = system(cmd);
    
    % Split the large registry string into individual registry entries using '\n\n' as a delimiter
    registryStrings = strsplit(largeRegistryString, '\n\n');
    % Predefined list of Bluetooth device names to search for
    string_list = {'ESP32-BT-Slave', '01_CLASSROOM_EDU_EXO_PRO', '02_CLASSROOM_EDU_EXO_PRO', ...
                   '03_CLASSROOM_EDU_EXO_PRO', '04_CLASSROOM_EDU_EXO_PRO', '05_CLASSROOM_EDU_EXO_PRO', ...
                  '06_CLASSROOM_EDU_EXO_PRO', '07_CLASSROOM_EDU_EXO_PRO', '08_CLASSROOM_EDU_EXO_PRO'};
    BT_devices.COMport = string.empty();
    BT_devices.friendlyName = string.empty();
    j=1;
    for i=1:length(string_list)
        % find the index of the friendly Name in string_list(i) in the registry string 
        friendlyName_indices = find(contains(registryStrings, string_list(i)));
    
        % Extract the Device Number from the registry string
        device_number_pattern = 'Dev_\w+';
        Dev_Number = regexp(registryStrings(friendlyName_indices), device_number_pattern, 'match', 'once');
        Dev_Number = strrep(Dev_Number, 'Dev_', '');  % or use erase function
    
        % find the index of the COM port using the Device Number
        COM_indices = find(contains(registryStrings, Dev_Number));
        % remove duplicate indices 
        COM_indices = COM_indices(~ismember(COM_indices,friendlyName_indices));
    
        % Extract Com Port Number from registry string
        com_port_pattern = '\(COM\d+\)';
        comPort = regexp(registryStrings(COM_indices), com_port_pattern, 'match', 'once');
        comPort = erase(comPort, {'(',')'});
    

        % Match Friendly Name to Comport in a struct
        if(~isempty(comPort))
            BT_devices.COMport(j) = comPort;
            BT_devices.friendlyName(j) = string_list(i);
            j =j+1;
        end
    end
    

end