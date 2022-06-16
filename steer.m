function X = steer(q_random, q_near, val, eps)

%% a function that takes the coordinates of the random point, closest node in the explored tree to the random point, val and maximum line distance to steer towards q_near with maximum step size of eps
   qnew = [0 0];
   
   
   if val >= eps
       qnew(1) = q_near(1) + ((q_random(1)-q_near(1))*eps)/distance(q_random,q_near);
       qnew(2) = q_near(2) + ((q_random(2)-q_near(2))*eps)/distance(q_random,q_near);
   else
       qnew(1) = q_random(1);
       qnew(2) = q_random(2);
   end   
   X = [qnew(1), qnew(2)];
end