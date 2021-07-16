/*
  asyncHttpClient.ino - FreeDs http async client
  Derivador de excedentes para ESP32 DEV Kit // Wifi Kit 32
  
  Copyright (C) 2020 Pablo Zerón (https://github.com/pablozg/freeds)

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

uint8_t shellySensor = 1;

void runAsyncClient()
{
  // INFOV(PSTR("\nFree Heap: %d bytes, Fragmentation: %.02f %%\n"), ESP.getFreeHeap(), getFragmentation());
  
  static char url[250];

  switch (config.wversion)
  {
    case SOLAX_V2_LOCAL: // Solax v2 local mode
      // strcpy(url, "POST /?optType=ReadRealTimeData HTTP/1.1\r\nHost: 5.8.8.8\r\nConnection: close\r\nContent-Length: 0\r\nAccept: /*/\r\nContent-Type: application/x-www-form-urlencoded\r\nX-Requested-With: com.solaxcloud.starter\r\n\r\n");
      // strcpy(url, "POST /?optType=ReadRealTimeData&pwd=admin HTTP/1.1\r\nHost: 5.8.8.8\r\nConnection: close\r\nContent-Length: 0\r\nAccept: /*/\r\nContent-Type: application/x-www-form-urlencoded\r\nX-Requested-With: com.solaxcloud.starter\r\n\r\n");
      
      sprintf(url, "POST /?optType=ReadRealTimeData&pwd=admin HTTP/1.1\r\nHost: %s\r\nConnection: close\r\nContent-Length: 0\r\nAccept: /*/\r\nContent-Type: application/x-www-form-urlencoded\r\nX-Requested-With: com.solaxcloud.starter\r\n\r\n", config.sensor_ip);
      // sprintf(url, "POST /?optType=ReadRealTimeData&pwd=admin\r\n\r\n");
      break;
    case SOLAX_V1: // Solax v1
      sprintf(url, "GET /api/realTimeData.htm HTTP/1.1\r\nHost: %s\r\nConnection: close\r\n\r\n", config.sensor_ip);
      break;
    case WIBEEE: // Wibee
      sprintf(url, "GET /en/status.xml HTTP/1.1\r\nHost: %s\r\nConnection: close\r\n\r\n", config.sensor_ip);
      break;
    case SHELLY_EM: // Shelly EM
      sprintf(url, "GET /emeter/%d HTTP/1.1\r\nHost: %s\r\nConnection: close\r\n\r\n", (shellySensor - 1), config.sensor_ip);
      break;
    case FRONIUS_API: // Fronius
      sprintf(url, "GET /solar_api/v1/GetPowerFlowRealtimeData.fcgi HTTP/1.1\r\nHost: %s\r\nConnection: close\r\n\r\n", config.sensor_ip);
      break;
    case SLAVE_MODE: // Slave Freeds
      sprintf(url, "GET /masterdata HTTP/1.1\r\nHost: %s\r\nConnection: close\r\n\r\n", config.sensor_ip);
      // sprintf(url, "POST /masterdata HTTP/1.1\r\nHost: %s\r\nUser-Agent: freeDS\r\nAccept: */*\r\nConnection: close\r\nContent-Type: application/json\r\nContent-Length: 21\r\n\r\n{\"command\":\"getData\"}\r\n", config.sensor_ip);
      // sprintf(url, "POST /masterdata HTTP/1.1\r\nHost: %s\r\nContent-Type: application/json\r\nContent-Length: 21\r\n\r\n{\"command\":\"getData\"}\r\n", config.sensor_ip);
      break;
  }

  if (httpRequest[4].readyState() == readyStateUnsent || httpRequest[4].readyState() == readyStateDone) {
        // INFOV("URL: %s\n", url);
        httpRequest[4].open(config.sensor_ip, 80, url);
  }
}

void parseSocket(uint8_t socket, bool State)
{
  static char url[250];

  sprintf(url, "GET /cm?user=admin&password=%s&cmnd=Power%%20%d HTTP/1.1\r\nHost: %s\r\nConnection: close\r\n\r\n", config.socketPass[socket - 1], State, config.socketIP[socket - 1]);

  INFOV("parseSocket Relay: %d, state %s, Pass: %s, IP: %s, URL: %s\n", socket, State ? "ON" : "OFF", config.socketPass[socket - 1], config.socketIP[socket - 1], url);

  if (httpRequest[socket - 1].readyState() == readyStateUnsent || httpRequest[socket - 1].readyState() == readyStateDone) {
    httpRequest[socket - 1].open(config.socketIP[socket - 1], 80, url);    
  }  
}

void sendRequest()
{
  static char url[250];
  if( httpRequest[4].readyState() == readyStateUnsent || httpRequest[4].readyState() == readyStateDone) {
      sprintf(url, "GET /masterdata HTTP/1.1\r\nHost: %s\r\nConnection: close\r\n\r\n", config.sensor_ip);
      INFOV("URL: %s\n", url);
      httpRequest[4].open(config.sensor_ip, 80, url);
  }
}

void sendRequest2()
{
  static char url[250];
  if (httpRequest[4].readyState() == readyStateUnsent || httpRequest[4].readyState() == readyStateDone) {
      sprintf(url, "GET /cm?user=admin&password=%s&cmnd=Status HTTP/1.1\r\nHost: %s\r\nConnection: close\r\n\r\n", config.socketPass[0], config.socketIP[0]);
      INFOV("URL: %s\n", url);
      httpRequest[4].open(config.socketIP[0], 80, url);
  }
}

void requestCB(void* optParm, asyncHttpClient* request, int readyState)
{
  (void) optParm;
  
  if (readyState == readyStateDone) {
    //Serial.printf("RequestCB -> %s\n", request->responseText());
    //request->setDebug(false);
    
    switch (config.wversion)
    {
      case SOLAX_V2_LOCAL: // Solax v2 local mode
        if (config.solaxVersion == 2) { parseJsonv2local(request->responseText()); }
        else { parseJsonv3local(request->responseText()); }
        break;
      case SOLAX_V1: // Solax v1
        parseJsonv1(request->responseText());
        break;
      case WIBEEE: // Wibee
        parseWibeee(request->responseText());
        break;
      case SHELLY_EM: // Shelly EM
        parseShellyEM(request->responseText(), shellySensor);
        shellySensor++;
        if (shellySensor > 2) { shellySensor = 1; }
        break;
      case SLAVE_MODE:
        parseMasterFreeDs(request->responseText());
        break;
      case FRONIUS_API: // Fronius
        parseJsonFronius(request->responseText());
        break;
    }
  }
}

void requestCBSocket(void* optParm, asyncHttpClient* request, int readyState)
{
  (void) optParm;
  if (readyState == readyStateDone) {
      Serial.printf("RequestCBSocket -> %s\n", request->responseText());
      //request->setDebug(false);
  }
}

// Function to implement strstr() function using KMP algorithm
inline char* strstr(char* X, const char* Y, int m, int n)
{
	// Base Case 1: Y is NULL or empty
	if (*Y == '\0' || n == 0)
		return X;

	// Base Case 2: X is NULL or X's length is less than that of Y's
	if (*X == '\0' || n > m)
		return NULL;

	// next[i] stores the index of next best partial match
	int next[n + 1];

	for (int i = 0; i < n + 1; i++)
		next[i] = 0;

	for (int i = 1; i < n; i++)
	{
		int j = next[i + 1];

		while (j > 0 && Y[j] != Y[i])
			j = next[j];

		if (j > 0 || Y[j] == Y[i])
			next[i + 1] = j + 1;
	}

	for (int i = 0, j = 0; i < m; i++)
	{
		if (*(X + i) == *(Y + j))
		{
			if (++j == n)
				return (X + i - j + 1);
		}
		else if (j > 0) {
			j = next[j];
			i--;	// // since i will be incremented in next iteration
		}
	}

	return NULL;
}