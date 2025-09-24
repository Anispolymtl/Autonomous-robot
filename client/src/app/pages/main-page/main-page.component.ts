// import { HttpErrorResponse } from '@angular/common/http';
import { Component } from '@angular/core';
// import { CommunicationService } from '@app/services/communication.service';
// import { Message } from '@common/message';
// import { BehaviorSubject } from 'rxjs';
import { RobotLoginComponent } from '@app/components/robot-login/robot-login.component';


@Component({
    selector: 'app-main-page',
    templateUrl: './main-page.component.html',
    styleUrls: ['./main-page.component.scss'],
    imports: [RobotLoginComponent],
})
export class MainPageComponent {

    // sendTimeToServer(): void {
    //     const newTimeMessage: Message = {
    //         title: 'Hello from the client',
    //         body: 'Time is : ' + new Date().toString(),
    //     };
    //     // Important de ne pas oublier "subscribe" ou l'appel ne sera jamais lancé puisque personne l'observe
    //     this.communicationService.basicPost(newTimeMessage).subscribe({
    //         next: (response) => {
    //             const responseString = `Le serveur a reçu la requête a retourné un code ${response.status} : ${response.statusText}`;
    //             this.message.next(responseString);
    //         },
    //         error: (err: HttpErrorResponse) => {
    //             const responseString = `Le serveur ne répond pas et a retourné : ${err.message}`;
    //             this.message.next(responseString);
    //         },
    //     });
    // }

    // getMessagesFromServer(): void {
    //     this.communicationService
    //         .basicGet()
    //         // Cette étape transforme l'objet Message en un seul string
    //         .pipe(
    //             map((message: Message) => {
    //                 return `${message.title} ${message.body}`;
    //             }),
    //         )
    //         .subscribe(this.message);
    // }
    // onIdentify(): void {
    //     this.identifyService.identifyRobot().subscribe({
    //         next: (res: any) => {
    //             this.message.next(`Réponse du robot : ${res.message}`);
    //         },
    //         error: (err: HttpErrorResponse) => {
    //             this.message.next(`Erreur lors de l’identification : ${err.message}`);
    //         },
    //     });
    // }
}
