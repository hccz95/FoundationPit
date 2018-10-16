//Assign exits for every agent based on the theory of GVD. 
int numDealtAgent = 0;
double factor, radius;
do
{
	for (int i = 0; i < numExit; ++i)
	{
		radius = Radius[i];
		if (numExitDealtAgent[i] < numAgentDistribution[i])
		{
			for (int j = 0; j < numAgent; ++j)
			{
				if (m_Vehicles[j]->m_iExitID != -1) continue;

				double distSq = Vec2DDistanceSq(m_vExits[i]->Pos(), m_Vehicles[j]->Pos());
				if (distSq <= radius*radius)
				{
					m_Vehicles[j]->m_iExitID = i;
					numDealtAgent++;
					numExitDealtAgent[i]++;
					if (numExitDealtAgent[i] >= numAgentDistribution[i]) break;
				}
			}
		}
		Radius[i] += Factor[i];
	}
} while (numDealtAgent < numAgent);
