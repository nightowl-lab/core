/**
 * 参数解析错误
 */
 export class InvalidArgumentsException extends Error {
    constructor(name: string) {
        super(`Failed to parse arguments, name: ${name}`);
    }
}