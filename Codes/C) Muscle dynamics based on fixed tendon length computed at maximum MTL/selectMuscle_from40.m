function rowNumber = selectMuscle_from40()
    % Define the list of muscles
    muscles = {
        'Acromiodeltoideus'
        'Anconeus'
        'Biceps Brachii'
        'Brachioradialis'
        'Brachialis'
        'Coracobrachialis'
        'Extensor Carpi Radialis'
        'Extensor Digitorum Communis 2'
        'Extensor Digitorum Communis 3'
        'Extensor Digitorum Communis 4'
        'Extensor Digitorum Communis 5'
        'Extensor Pollicis Longus 1'
        'Extensor Pollicis Longus 2'
        'Epitrochlearis'
        'Flexor Carpi Radialis'
        'Flexor Carpi Ulnaris'
        'Flexor Digitorum Profundus 1'
        'Flexor Digitorum Profundus 2'
        'Flexor Digitorum Profundus 3'
        'Flexor Digitorum Profundus 4'
        'Flexor Digitorum Profundus 5'
        'Flexor Digitorum Superficialis 2'
        'Flexor Digitorum Superficialis 3'
        'Flexor Digitorum Superficialis 4'
        'Flexor Digitorum Superficialis 5'
        'Infraspinatus' 
        'Palmaris Longus 1' 
        'Palmaris Longus 2'
        'Palmaris Longus 3'
        'Palmaris Longus 4' 
        'Palmaris Longus 5'
        'Pronator Teres'
        'Spinodeltoideus'
        'Supraspinatus'
        'Subscapularis'
        'Teres Major'
        'Triceps Brachii Lateralis'
        'Triceps Brachii Long'
        'Triceps Brachii Medial'
        'Teres Minor'
    };

    % Set the size of the list dialog window
    dialogSize = [300, 400]; % [width, height]

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
