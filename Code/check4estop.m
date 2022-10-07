% eStop
function check4eStop()
check = 0;
% key_press = waitforbuttonpress;
% val = double(get(gcf, 'CurrentCharacter'))
if app.estopbutton.Value == 1
%if (val == 115)
    check = 1;
    while (check == 1)
        input('eStop has been activated! Press enter to continue');
        check = 0;
    end
end
return
end