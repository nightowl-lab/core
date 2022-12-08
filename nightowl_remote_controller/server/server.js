import { WebSocketServer } from 'ws';
import { readFileSync } from 'fs'

/* 配置项 */
const port = 8000;
const errorCode = {
    "INVALID_ARGUMENTS": -1001,
    "NOT_LOGIN": -1002,
    "PERMISSION_DENIED": -1003,
    "TARGET_OFFLINE": -1004,

    "USERNAME_OR_PASSWORD_OR_ROLE_WRONG": -2001
};

/* 初始化WS服务器 */
const database = JSON.parse(readFileSync('./users.json'));
const server = new WebSocketServer({ port });
let clients = {};

function checkVehicleHasPermissionToSend(fromUserName, toUserName) {
    for (const user of database.users) {
        if (user.username === toUserName) {
            return user.allow.includes(fromUserName);
        }
    }
    return false;
}

function getUserInfo(username, password, role) {
    for (const user of database.users) {
        if (user.username === username && user.password === password && user.role === role) {
            return user;
        }
    }
    return undefined;
}

function sendMessage(from, to, callID, type, data)
{
    let newData = {
        'from': from,
        'callID': callID,
        'type': type,
        'data': data
    };
    to.send(JSON.stringify(newData));
}

server.on('connection', (connection, request) => {
    let user = undefined;
    const ip = request.socket.remoteAddress;

    console.log(`[Server] [${ip}] Client connected!`);

    /* 心跳处理 */
    connection.isAlive = true;
    connection.offlineCount = 0;
    connection.on('pong', () => {
        connection.isAlive = true;
        connection.offlineCount = 0;
    });

    connection.on('close', () => {
        console.log(`[Server] [${ip}] Client disconnected!`);
        if (user !== undefined) {
            delete clients[user.username];
        }
    });

    connection.on('message', (data) => {
        /* 数据校验 */
        let jsonData;
        try {
            jsonData = JSON.parse(data);
            if (typeof jsonData['to'] !== 'string' || typeof jsonData['callID'] !== 'number' ||
                typeof jsonData['type'] !== 'string' || typeof jsonData['data'] === 'undefined') {
                throw "";
            }
        } catch (e) {
            console.log(`[Server] [${ip}] Failed to parse json from client!`);
            sendMessage('server', connection, 0, 'response', { code: errorCode.INVALID_ARGUMENTS });
            return;
        }
        /* 处理登录请求 */
        if (jsonData.to === 'server') {
            if (jsonData.type === 'login') {
                if (typeof jsonData['data'] !== 'object' || typeof jsonData['data']['username'] !== 'string' ||
                    typeof jsonData['data']['password'] !== 'string' || typeof jsonData['data']['role'] !== 'string') {
                    console.log(`[Server] [${ip}] Failed to parse login arguments!`);
                    sendMessage('server', connection, 0, 'response', { code: errorCode.INVALID_ARGUMENTS });
                    return;
                }
                let newUser = getUserInfo(jsonData.data.username, jsonData.data.password, jsonData.data.role);
                if (newUser === undefined) {
                    console.log(`[Server] [${ip}] Wrong username or password or role to login`);
                    sendMessage('server', connection, jsonData.callID, 'response', { code: errorCode.USERNAME_OR_PASSWORD_OR_ROLE_WRONG });
                    return;
                }
                /* 登录成功 */
                if (user !== undefined) {
                    delete clients[user.username];
                }
                clients[newUser.username] = connection;
                user = newUser;
                sendMessage('server', connection, jsonData.callID, 'response', { code: 0 });
                console.log(`[Server] [${ip}] Login successful!`);
                return;
            }
            console.log(`[Server] [${ip}] Wrong type: ${jsonData.type}`);
            sendMessage('server', connection, 0, 'response', { code: errorCode.INVALID_ARGUMENTS });
            return;
        }
        /* 转发请求 */
        if (user === undefined) {
            console.log(`[Server] [${ip}] Client is not login`);
            sendMessage('server', connection, jsonData.callID, 'response', { code: errorCode.NOT_LOGIN });
            return;
        } else {
            const logUserName = user.role + " / " + user.username;
            /* 检查权限 */
            if ((user.role === 'USER' && !user.allow.includes(jsonData.to)) ||
                (user.role === 'VEHICLE' && !checkVehicleHasPermissionToSend(user.username, jsonData.to))) {
                console.log(`[Server] [${logUserName}] Client hasn't permission to send to target`);
                sendMessage('server', connection, jsonData.callID, 'response', { code: errorCode.PERMISSION_DENIED });
                return;
            }
            /* 开始转发 */
            console.log(`[Forward] [${logUserName}] From ${user.username} to ${jsonData.to}, type: ${jsonData.type}, callID: ${jsonData.callID}`);
            /* 检查目标是否在线 */
            if (!(jsonData.to in clients)) {
                console.log(`[Forward] [${logUserName}] Target is offline`);
                sendMessage('server', connection, jsonData.callID, 'response', { code: errorCode.TARGET_OFFLINE });
                return;
            }
            sendMessage(user.username, clients[jsonData.to], jsonData.callID, jsonData.type, jsonData.data);
        }
    });
});

/* 心跳检测 */
setInterval(() => {
    server.clients.forEach((ws) => {
        if (ws.isAlive === false) {
            ws.offlineCount++;
            if (ws.offlineCount == 10) {
                return ws.terminate();
            }
        }

        ws.isAlive = false;
        ws.ping();
    });
}, 1000);

console.log(`[Server] Listening at ${port}`);