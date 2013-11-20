% Returns the scene type (living room, starbucks, subway, etc) from the
% scene (living_room_0002k, office_0013, etc).
%
% Args:
%   scene - the scene name: [sceneType]_[sceneNumber]
% 
% Returns:
%   sceneType - the name of the scene type.
function sceneType = get_scene_type_from_scene(scene)
  ind = regexp(scene, '\d+\w?');
  sceneType = scene(1:ind-2);
end