function [quats] = fixquaternionsign(quats)
for t=2:size(quats,1)
  dot = quats(t-1,:)*quats(t,:)';
  if (dot<0)
    quats(t,:) = -quats(t,:);
  end
end