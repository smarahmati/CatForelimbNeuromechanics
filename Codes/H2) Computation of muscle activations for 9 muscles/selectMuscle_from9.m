function rowNumber = selectMuscle_from9()
    % Define the list of muscles
    muscles = {
        'Shoulder Protractor'
    	'Elbow Extensor'
    	'Biceps Brachii'
    	'Elbow Flexor'
    	'Elbow Flexor-Wrist Dorsiflexor'
    	'Wrist Dorsiflexor'
    	'Wrist Plantarflexor'
    	'Shoulder Retractor'
    	'Triceps Brachii Long'
    };

    % Set the size of the list dialog window
    dialogSize = [300, 150]; % [width, height]

    % Create a dialog box for user to select a muscle
    [selection, ok] = listdlg('PromptString', 'Select a muscle:', ...
                              'SelectionMode', 'single', ...
                              'ListSize', dialogSize, ...
                              'ListString', muscles);

    % Check if the user made a selection
    if ok
        % Get the selected muscle and its row number
        selectedMuscle = muscles{selection};
        rowNumber = selection;
        
        % Display the selected muscle and its row number
        fprintf('You selected: %s\n', selectedMuscle);
        fprintf('Row of the selected muscle: %d\n', rowNumber);
    else
        % If no selection was made, return an empty array
        rowNumber = [];
        fprintf('No muscle selected.\n');
    end
end
