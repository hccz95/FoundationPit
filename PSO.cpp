//Particle Swarm Optimization
void SimOptimization::PSO_ExitOpt()
{
	struct Particle			// Particle
	{
		std::vector<float> pos;			
		std::vector<Vector2D> exitPos;
		std::vector<float> bestPos;
		double curFitness = 0;
		double bestFitness = 0;
	};
	std::vector<Particle> swarms;	//Swarms
	int numParticle = 30;

	m_vBestPosWithNExits.clear();
	m_vBestExitsPosWithNExits.clear();
	m_vBestFitWithNExits.clear();
	m_vBestAgentStatsWithNExits.clear();
	m_BestExitStatsWithNExits.clear();

	int numWalls = CGlobal::g_GameWorld->Walls().size();
	for (CGlobal::m_gNumExits = 1; CGlobal::m_gNumExits <= numWalls*2;++CGlobal::m_gNumExits)
	{
		// Clear the data structures
		swarms.clear();
		m_CurAgentStats.Reset();
		m_CurExitStats.clear();
		m_BestExitStats.clear();
		m_vParticleAgentStats.clear();
		m_vParticleExitStats.clear();
		// Define some local variables
		std::vector<float> globalBestPos;
		std::vector<Vector2D> globalBestExitPos;
		double globalBestFit = 0;

		// Initialize exits distribution
		// Restrictions: the position of exits can¡¯t overlap with slope
		for (int j = 0; j < CGlobal::m_gNumExits; ++j)
		{
			m_CurExitStats.push_back(ExitStats());
			m_BestExitStats.push_back(ExitStats());
			globalBestExitPos.push_back(Vector2D(0, 0));
			float pos;
			do
			{
				pos = RandFloat();
			} while (!FloatToExit(pos, globalBestExitPos[j]));
			globalBestPos.push_back(pos);
		}

		std::vector<Exit*>* exits = CGlobal::g_GameWorld->GetExits();
		for (unsigned int i = 0; i < (*exits).size(); ++i)
		{
			delete (*exits)[i];
		}
		CGlobal::g_GameWorld->m_vExits.clear();

		for (int i = 0; i < numParticle; ++i)		// enumerate every particle
		{
			Particle particle;
			for (int j = 0; j < CGlobal::m_gNumExits; ++j)
			{
				particle.exitPos.push_back(Vector2D(0, 0));

				float pos;
				do
				{
					pos = RandFloat();
				} while (!FloatToExit(pos, particle.exitPos[j]));
				particle.pos.push_back(pos);
				particle.bestPos.push_back(pos);
			}
			//particle.curFitness = 0;
			//particle.bestFitness = 0;
			swarms.push_back(particle);
		}

		// set parameters
		float c1 = 3;
		float c2 = 5;

		// the process of optimization
		for (int gen = 0; gen < 10; gen++)
		{
			for (int particleID = 0; particleID < numParticle; particleID++)
			{ 
				// erase the exit position of last swarm
				for (unsigned int i = 0; i < (*exits).size(); ++i)
					delete (*exits)[i];
				CGlobal::g_GameWorld->m_vExits.clear();

				Particle* curParticle = &swarms[particleID];
				for (int j = 0; j < CGlobal::m_gNumExits; ++j)
				{
					// update the position
					float pos = curParticle->pos[j] + c1*RandFloat()*(curParticle->bestPos[j] - curParticle->pos[j]) +
						c2*RandFloat()*(globalBestPos[j] - curParticle->pos[j]);
					if (FloatToExit(pos, curParticle->exitPos[j])) curParticle->pos[j] = pos;
					else
					{
						j--;
						continue;
					}
				}

				// simulation estimation
				for (int j = 0; j < CGlobal::m_gNumExits; ++j)
				{
					CGlobal::g_GameWorld->AddExit(curParticle->exitPos[j]);
				}
				SimulationOnCollapseSet();

				// current assessment
				curParticle->curFitness = m_CurAgentStats.m_dPercentAlive;
				if (curParticle->curFitness > curParticle->bestFitness)
				{
					curParticle->bestFitness = curParticle->curFitness;
					for (int j = 0; j < CGlobal::m_gNumExits; ++j)
					{
						curParticle->bestPos[j] = curParticle->pos[j];
					}
				}
				if (curParticle->curFitness > globalBestFit)
				{
					globalBestFit = curParticle->curFitness;
					for (int j = 0; j < CGlobal::m_gNumExits; ++j)
					{
						globalBestPos[j] = curParticle->pos[j];
						globalBestExitPos[j] = curParticle->exitPos[j];

						float pos;
						do
						{
							pos = RandFloat();
							curParticle->pos[j] = pos;
						} while (!FloatToExit(pos, curParticle->exitPos[j]));

						m_BestExitStats[j] = m_CurExitStats[j];
					}

					m_BestAgentStats = m_CurAgentStats;
					// get rid of the slope
				}

			}
			m_vParticleAgentStats.push_back(m_BestAgentStats);
			m_vParticleExitStats.push_back(m_BestExitStats);
			// get rid of the slope

			if (gen >= 1)
			{
				double lastBestFit = m_vParticleAgentStats[gen - 1].m_dPercentAlive;
				double thisBestFit = m_vParticleAgentStats[gen].m_dPercentAlive;
				if (abs(lastBestFit - thisBestFit) < 0.0001)	// terminate condition
					break;
			}

			if (c2>3 && c1<5)
			{
				c2 -= 0.4; c1 += 0.4;
			}
		}

		// save optimal exit distribution
		m_vBestAgentStatsWithNExits.push_back(m_BestAgentStats);
		m_BestExitStatsWithNExits.push_back(m_BestExitStats);
		m_vBestPosWithNExits.push_back(globalBestPos);
		m_vBestExitsPosWithNExits.push_back(globalBestExitPos);
		m_vBestFitWithNExits.push_back(globalBestFit);
	}

	WriteStatToFile();
}
