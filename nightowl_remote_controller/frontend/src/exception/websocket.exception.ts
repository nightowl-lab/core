/**
 * WebSocket错误
 */
export class WebSocketException extends Error {
    constructor(public code: number) {
        super(`Failed to make WebSocket RPC Call, code: ${code}`);
    }
}