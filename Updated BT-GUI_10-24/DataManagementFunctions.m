classdef DataManagementFunctions
    methods (Static)
        function saveData(data, filename)
            % Implementation for saving data to .mat file
            save(filename, 'data');
        end

        function data = loadData(filename)
            % Implementation for loading data from .mat file
            loadedData = load(filename);
            data = loadedData.data;
        end
    end
end