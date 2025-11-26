process.env.ROS_DOMAIN_ID = '66';

import { Injectable, Logger } from '@nestjs/common';
import * as rclnodejs from 'rclnodejs';

@Injectable()
export class CodeEditorService {
    private readonly logger = new Logger(CodeEditorService.name);
    private node: rclnodejs.Node;

    private readonly srv = (rclnodejs.require('limo_interfaces') as any).srv;

    private getCodeClient: rclnodejs.Client<any>;
    private saveCodeClient: rclnodejs.Client<any>;

    // ============================================================
    //              INITIALISATION DES CLIENTS ROS2
    // ============================================================
    initCodeEditorService() {
        // IMPORTANT → spin obligatoire
        this.node = new rclnodejs.Node('code_editor_client_backend');
        this.node.spin();

        this.getCodeClient = this.node.createClient(
        this.srv.GetScript,
        '/get_robot_script'
        );

        this.saveCodeClient = this.node.createClient(
        this.srv.SaveScript,
        '/save_robot_script'
        );

        this.logger.log('Clients ROS2 du Code Editor initialisés');
    }

    // ============================================================
    //                     GET SCRIPT
    // ============================================================
    async getCode(): Promise<{ success: boolean; message: string; code?: string }> {
        console.log('Get code')
        await this.getCodeClient.waitForService(2000).catch(() => {});
        if (!this.getCodeClient.isServiceServerAvailable()) {
            this.logger.error('Client ROS2 GetScript non prêt');
            return { success: false, message: 'Service ROS2 non disponible' };
        }

        return new Promise((resolve) => {
            const request = new this.srv.GetScript.Request();

            this.getCodeClient.sendRequest(request, (response) => {
                console.log(response.code)
                if (!response) {
                    resolve({
                        success: false,
                        message: 'Aucune réponse du service ROS2',
                    });
                } else {
                    resolve({
                        success: response.success,
                        message: response.message,
                        code: response.code,
                    });
                }
            });
        });
    }

    // ============================================================
    //                     SAVE SCRIPT
    // ============================================================
    async saveCode(newCode: string): Promise<{ success: boolean; message: string }> {
        await this.saveCodeClient.waitForService(3000).catch(() => {});
        if (!this.saveCodeClient.isServiceServerAvailable()) {
            this.logger.error('Client ROS2 SaveScript non prêt');
            return { success: false, message: 'Service ROS2 non disponible' };
        }

        return new Promise((resolve) => {
            const request = new this.srv.SaveScript.Request();
            request.new_code = newCode;

            this.saveCodeClient.sendRequest(request, (response) => {
                if (!response) {
                    resolve({
                        success: false,
                        message: 'Aucune réponse du service ROS2',
                    });
                } else {
                    resolve({
                        success: response.success,
                        message: response.message,
                    });
                }
            });
        });
    }
}
