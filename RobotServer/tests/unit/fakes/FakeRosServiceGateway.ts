export interface RecordedServiceCall {
  type: string;
  serviceName: string;
  request: unknown;
}

export class FakeRosServiceGateway {
  calls: RecordedServiceCall[] = [];
  private readonly responses = new Map<string, unknown>();
  private readonly rejections = new Map<string, Error>();

  async call<TReq, TRes>(type: string, serviceName: string, request: TReq): Promise<TRes> {
    this.calls.push({ type, serviceName, request });

    const rejection = this.rejections.get(serviceName);
    if (rejection) {
      throw rejection;
    }

    return (this.responses.get(serviceName) as TRes) ?? (undefined as unknown as TRes);
  }

  /** Test helper: what call() resolves with for a given service name. */
  setResponse(serviceName: string, response: unknown): void {
    this.responses.set(serviceName, response);
  }

  /** Test helper: makes call() reject for a given service name. */
  setRejection(serviceName: string, error: Error): void {
    this.rejections.set(serviceName, error);
  }
}
